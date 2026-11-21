package mapper

import (
	"container/heap"
	"errors"
	"fmt"
	"math"
	"sort"
	"time"

	"github.com/GoMudEngine/GoMud/internal/rooms"
	"github.com/GoMudEngine/GoMud/internal/util"
	lru "github.com/hashicorp/golang-lru/v2"
)

var (
	ErrPathNotFound    = errors.New(`path not found`)
	ErrPathDestMatch   = errors.New(`path destination is same as source`)
	ErrPathBlocked     = errors.New(`path blocked by constraints`)
	ErrPathSearchLimit = errors.New(`path search limit exceeded`)
	pathCache, _       = lru.New[pathCacheKey, pathCacheValue](128)
)

// RoomExit identifies a single named exit of a specific room.
type RoomExit struct {
	RoomId   int
	ExitName string
}

// PathConstraints describes rooms and exits a path query must avoid.
// The zero value imposes no constraints.
type PathConstraints struct {
	BlockedRooms map[int]struct{}
	BlockedExits map[RoomExit]struct{}
}

// BlockRoom adds a room the path may not enter, and returns the
// constraints for chaining.
func (c *PathConstraints) BlockRoom(roomId int) *PathConstraints {
	if c.BlockedRooms == nil {
		c.BlockedRooms = make(map[int]struct{})
	}
	c.BlockedRooms[roomId] = struct{}{}
	return c
}

// BlockExit adds a single named exit of a room the path may not take,
// and returns the constraints for chaining.
func (c *PathConstraints) BlockExit(roomId int, exitName string) *PathConstraints {
	if c.BlockedExits == nil {
		c.BlockedExits = make(map[RoomExit]struct{})
	}
	c.BlockedExits[RoomExit{RoomId: roomId, ExitName: exitName}] = struct{}{}
	return c
}

// blocksRoom reports whether a room is off-limits. Nil-safe.
func (c *PathConstraints) blocksRoom(roomId int) bool {
	if c == nil {
		return false
	}
	_, ok := c.BlockedRooms[roomId]
	return ok
}

// blocksExit reports whether a specific exit of a room is off-limits. Nil-safe.
func (c *PathConstraints) blocksExit(roomId int, exitName string) bool {
	if c == nil {
		return false
	}
	_, ok := c.BlockedExits[RoomExit{RoomId: roomId, ExitName: exitName}]
	return ok
}

type pathCacheKey struct {
	startRoomId int
	endRoomId   int
}

type pathCacheValue struct {
	steps []pathStep
	err   error
}

// pathStep is one move: take ExitName to arrive in RoomID.
type pathStep struct {
	exitName string
	roomId   int
	waypoint bool
}

func (p pathStep) ExitName() string {
	return p.exitName
}

func (p pathStep) RoomId() int {
	return p.roomId
}

func (p pathStep) Waypoint() bool {
	return p.waypoint
}

// internal struct to record how we reached each node
type prevInfo struct {
	prevRoom int    // the room we came from
	viaExit  string // the exit name we used
}

// A nodeRecord is what we store in the priority queue.
type nodeRecord struct {
	roomId int
	fScore float64
	index  int
}

type priorityQueue []*nodeRecord

func (pq priorityQueue) Len() int { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool {
	// Stable tie-break on roomId so equal-cost routes resolve deterministically.
	if pq[i].fScore == pq[j].fScore {
		return pq[i].roomId < pq[j].roomId
	}
	return pq[i].fScore < pq[j].fScore
}
func (pq priorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *priorityQueue) Push(x interface{}) {
	n := x.(*nodeRecord)
	n.index = len(*pq)
	*pq = append(*pq, n)
}
func (pq *priorityQueue) Pop() interface{} {
	old := *pq
	n := old[len(old)-1]
	old[len(old)-1] = nil // avoid mem leak
	n.index = -1
	*pq = old[:len(old)-1]
	return n
}
func (pq *priorityQueue) update(n *nodeRecord, newF float64) {
	n.fScore = newF
	heap.Fix(pq, n.index)
}

// heuristic is the 3D Manhattan distance.
// This should be redone - rooms can be one space away exit-wise, but 2+ spacex in grid space.
// Example: Frostfang city walls
func (r *mapper) heuristic(a, b int) float64 {
	ax, ay, az, _ := r.GetCoordinates(a)
	bx, by, bz, _ := r.GetCoordinates(b)
	return math.Abs(float64(ax-bx)) +
		math.Abs(float64(ay-by)) +
		math.Abs(float64(az-bz))
}

// FindPath returns the sequence of ExitName/RoomID steps from startRoom to goalRoom.
func (r *mapper) findPath(startRoom, goalRoom int) ([]pathStep, error) {
	return r.findPathConstrained(startRoom, goalRoom, nil)
}

// findPathConstrained is findPath with optional constraints: blocked rooms may
// not be entered and blocked exits may not be taken. A nil constraints pointer
// behaves exactly like findPath.
func (r *mapper) findPathConstrained(startRoom, goalRoom int, constraints *PathConstraints) ([]pathStep, error) {

	if startRoom == goalRoom {
		return nil, ErrPathDestMatch
	}

	// sanity check
	if _, ok := r.crawledRooms[startRoom]; !ok {
		return nil, ErrRoomNotFound
	}
	if _, ok := r.crawledRooms[goalRoom]; !ok {
		return nil, ErrRoomNotFound
	}

	// A blocked start or goal can never yield a route.
	if constraints.blocksRoom(startRoom) || constraints.blocksRoom(goalRoom) {
		return nil, ErrPathBlocked
	}

	// cameFrom holds, for each room, how we got there.
	cameFrom := make(map[int]prevInfo, len(r.crawledRooms))

	// gScore: cost from start to here; fScore = gScore + heuristic
	gScore := make(map[int]float64, len(r.crawledRooms))
	fScore := make(map[int]float64, len(r.crawledRooms))
	for id := range r.crawledRooms {
		gScore[id] = math.Inf(1)
		fScore[id] = math.Inf(1)
	}
	gScore[startRoom] = 0
	fScore[startRoom] = r.heuristic(startRoom, goalRoom)

	// open set as a priority queue
	openSet := make(priorityQueue, 0, len(r.crawledRooms))
	heap.Init(&openSet)
	heap.Push(&openSet, &nodeRecord{roomId: startRoom, fScore: fScore[startRoom]})
	inOpen := map[int]*nodeRecord{startRoom: openSet[0]}

	// Hard bound on expansions. Costs are uniform positive integers and every
	// (re)push of a room strictly lowers its integer gScore, so a room with n
	// rooms in the graph can be expanded well under n*n times. The cap only
	// guards against pathological states; it never triggers on a healthy graph.
	maxExpansions := len(r.crawledRooms)*len(r.crawledRooms) + len(r.crawledRooms) + 1
	expansions := 0

	for openSet.Len() > 0 {
		current := heap.Pop(&openSet).(*nodeRecord)
		delete(inOpen, current.roomId)

		expansions++
		if expansions > maxExpansions {
			return nil, ErrPathSearchLimit
		}

		// reached goal!
		if current.roomId == goalRoom {
			// reconstruct path
			var path []pathStep
			cur := goalRoom
			for cur != startRoom {
				info := cameFrom[cur]

				// record the exit name and the room we arrived in
				path = append(path, pathStep{exitName: info.viaExit, roomId: cur})
				cur = info.prevRoom
			}

			pathLen := len(path)

			if pathLen > 0 {
				// reverse
				for i := 0; i < pathLen/2; i++ {
					j := len(path) - 1 - i
					path[i], path[j] = path[j], path[i]
				}

				// Mark the final room as the waypoint
				path[pathLen-1].waypoint = true
			}
			return path, nil
		}

		// expand neighbors in sorted exit order so equal-cost routes are
		// explored (and selected) deterministically.
		node := r.crawledRooms[current.roomId]
		exitNames := make([]string, 0, len(node.Exits))
		for exitName := range node.Exits {
			exitNames = append(exitNames, exitName)
		}
		sort.Strings(exitNames)

		for _, exitName := range exitNames {
			exitInfo := node.Exits[exitName]
			neighbor := exitInfo.RoomId

			if constraints.blocksExit(current.roomId, exitName) {
				continue
			}
			if constraints.blocksRoom(neighbor) {
				continue
			}
			// Skip dangling exits that point at rooms the mapper doesn't know.
			if _, ok := r.crawledRooms[neighbor]; !ok {
				continue
			}

			tentativeG := gScore[current.roomId] + 1 // uniform cost

			if tentativeG < gScore[neighbor] {
				// this is a better path to neighbor
				cameFrom[neighbor] = prevInfo{
					prevRoom: current.roomId,
					viaExit:  exitName,
				}
				gScore[neighbor] = tentativeG
				fScore[neighbor] = tentativeG + r.heuristic(neighbor, goalRoom)

				if nr, ok := inOpen[neighbor]; ok {
					openSet.update(nr, fScore[neighbor])
				} else {
					nr := &nodeRecord{roomId: neighbor, fScore: fScore[neighbor]}
					heap.Push(&openSet, nr)
					inOpen[neighbor] = nr
				}
			}
		}
	}

	return nil, ErrPathNotFound
}

func GetPath(startRoomId int, endRoomId ...int) ([]pathStep, error) {

	start := time.Now()
	defer func() {
		util.TrackTime(`mapper.GetPath()`, time.Since(start).Seconds())
	}()

	if len(endRoomId) == 0 {
		return []pathStep{}, fmt.Errorf("%d => %d (endRoom not found): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	startRoom := rooms.LoadRoom(startRoomId)
	if startRoom == nil {
		return []pathStep{}, fmt.Errorf("%d => %d (startRoom  not found): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	m := GetMapper(startRoom.RoomId)
	if m == nil {
		return []pathStep{}, fmt.Errorf("%d => %d (mapper not fond): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	return getPath(m, startRoomId, nil, endRoomId...)
}

// GetPathConstrained behaves like GetPath, but the route may not enter any
// blocked room nor take any blocked exit named in constraints. Constrained
// queries never read from or write to the shared path cache, so they neither
// reuse unconstrained results nor pollute later unconstrained queries.
func GetPathConstrained(startRoomId int, constraints PathConstraints, endRoomId ...int) ([]pathStep, error) {

	start := time.Now()
	defer func() {
		util.TrackTime(`mapper.GetPathConstrained()`, time.Since(start).Seconds())
	}()

	if len(endRoomId) == 0 {
		return []pathStep{}, fmt.Errorf("%d => %d (endRoom not found): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	startRoom := rooms.LoadRoom(startRoomId)
	if startRoom == nil {
		return []pathStep{}, fmt.Errorf("%d => %d (startRoom  not found): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	m := GetMapper(startRoom.RoomId)
	if m == nil {
		return []pathStep{}, fmt.Errorf("%d => %d (mapper not fond): %w", startRoomId, endRoomId, ErrPathNotFound)
	}

	return getPath(m, startRoomId, &constraints, endRoomId...)
}

// getPath walks each destination leg in order through m. When constraints is
// nil the per-leg results are memoized in pathCache; constrained queries skip
// the cache entirely.
func getPath(m *mapper, startRoomId int, constraints *PathConstraints, endRoomId ...int) ([]pathStep, error) {

	useCache := constraints == nil

	cacheKey := pathCacheKey{}
	rNow := startRoomId
	finalPath := []pathStep{}
	for _, roomId := range endRoomId {

		if rNow == roomId { // Avoid repeating id's
			continue
		}

		if useCache {
			cacheKey.startRoomId = rNow
			cacheKey.endRoomId = roomId

			if pCache, ok := pathCache.Get(cacheKey); ok {
				pathCacheHits.Add(1)

				if pCache.err != nil {
					return pCache.steps, fmt.Errorf("%d => %d: %w", rNow, roomId, pCache.err)
				}

				finalPath = append(finalPath, pCache.steps...)
				rNow = roomId
				continue
			}

			pathCacheMisses.Add(1)
		}

		if !m.HasRoom(roomId) {
			err := fmt.Errorf("%d => %d (room not in mapper): %w", rNow, roomId, ErrPathNotFound)
			if useCache {
				pathCache.Add(cacheKey, pathCacheValue{steps: nil, err: err})
			}
			return []pathStep{}, err
		}

		p, err := m.findPathConstrained(rNow, roomId, constraints)
		if err != nil {
			if useCache {
				pathCache.Add(cacheKey, pathCacheValue{steps: p, err: err})
			}
			return []pathStep{}, fmt.Errorf("%d => %d: %w", rNow, roomId, err)
		}

		if useCache {
			// Add to LRU cache
			pathCache.Add(cacheKey, pathCacheValue{steps: p, err: nil})
		}

		finalPath = append(finalPath, p...)
		rNow = roomId
	}

	// If final path is empty, they mapped to the same room they are in.
	// This can occur if endRoomId differs from startRoomId, but endRoomId was actually
	// an alias equal to startRoomId
	if len(finalPath) == 0 {
		return finalPath, ErrPathDestMatch
	}

	return finalPath, nil
}
