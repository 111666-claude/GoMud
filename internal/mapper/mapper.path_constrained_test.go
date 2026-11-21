package mapper

import (
	"fmt"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// buildDetourMapper creates a fixed 5-room graph:
//
//	1 -east-> 2 -east-> 3
//	|                   |
//	south               south
//	|                   |
//	4 ------east------> 5
//
// The direct route 1->3 is east/east through room 2; the detour is
// south/east/north through rooms 4 and 5.
func buildDetourMapper() *mapper {
	m := NewMapper(1)
	m.crawledRooms = map[int]*mapNode{
		1: {RoomId: 1, Pos: positionDelta{x: 0, y: 0, z: 0}, Exits: map[string]nodeExit{"east": {RoomId: 2}, "south": {RoomId: 4}}},
		2: {RoomId: 2, Pos: positionDelta{x: 1, y: 0, z: 0}, Exits: map[string]nodeExit{"west": {RoomId: 1}, "east": {RoomId: 3}}},
		3: {RoomId: 3, Pos: positionDelta{x: 2, y: 0, z: 0}, Exits: map[string]nodeExit{"west": {RoomId: 2}, "south": {RoomId: 5}}},
		4: {RoomId: 4, Pos: positionDelta{x: 0, y: 1, z: 0}, Exits: map[string]nodeExit{"north": {RoomId: 1}, "east": {RoomId: 5}}},
		5: {RoomId: 5, Pos: positionDelta{x: 1, y: 1, z: 0}, Exits: map[string]nodeExit{"west": {RoomId: 4}, "north": {RoomId: 3}}},
	}
	return m
}

// buildDiamondMapper creates two equal-length routes 1->4:
// 1 -a-> 2 -c-> 4 and 1 -b-> 3 -d-> 4
func buildDiamondMapper() *mapper {
	m := NewMapper(1)
	m.crawledRooms = map[int]*mapNode{
		1: {RoomId: 1, Pos: positionDelta{x: 0, y: 0, z: 0}, Exits: map[string]nodeExit{"a": {RoomId: 2}, "b": {RoomId: 3}}},
		2: {RoomId: 2, Pos: positionDelta{x: 1, y: -1, z: 0}, Exits: map[string]nodeExit{"c": {RoomId: 4}}},
		3: {RoomId: 3, Pos: positionDelta{x: 1, y: 1, z: 0}, Exits: map[string]nodeExit{"d": {RoomId: 4}}},
		4: {RoomId: 4, Pos: positionDelta{x: 2, y: 0, z: 0}, Exits: map[string]nodeExit{}},
	}
	return m
}

func pathRooms(path []pathStep) []int {
	rooms := make([]int, len(path))
	for i, step := range path {
		rooms[i] = step.roomId
	}
	return rooms
}

func pathExits(path []pathStep) []string {
	exits := make([]string, len(path))
	for i, step := range path {
		exits[i] = step.exitName
	}
	return exits
}

func Test_findPathConstrained_NoConstraints(t *testing.T) {
	m := buildDetourMapper()
	path, err := m.findPathConstrained(1, 3, nil)
	require.NoError(t, err)
	assert.Equal(t, []string{"east", "east"}, pathExits(path))
	assert.Equal(t, []int{2, 3}, pathRooms(path))
	assert.True(t, path[len(path)-1].waypoint)
}

func Test_findPathConstrained_BlockedRoomDetour(t *testing.T) {
	m := buildDetourMapper()
	c := (&PathConstraints{}).BlockRoom(2)

	path, err := m.findPathConstrained(1, 3, c)
	require.NoError(t, err)
	assert.Equal(t, []string{"south", "east", "north"}, pathExits(path))
	assert.Equal(t, []int{4, 5, 3}, pathRooms(path))
	assert.True(t, path[len(path)-1].waypoint)
}

func Test_findPathConstrained_BlockedExitDetour(t *testing.T) {
	m := buildDetourMapper()

	// Blocking the exit out of the start room forces the detour.
	c := (&PathConstraints{}).BlockExit(1, "east")
	path, err := m.findPathConstrained(1, 3, c)
	require.NoError(t, err)
	assert.Equal(t, []string{"south", "east", "north"}, pathExits(path))

	// Blocking a mid-route exit forces the same detour.
	c = (&PathConstraints{}).BlockExit(2, "east")
	path, err = m.findPathConstrained(1, 3, c)
	require.NoError(t, err)
	assert.Equal(t, []string{"south", "east", "north"}, pathExits(path))

	// Blocking an exit that is not on any route changes nothing.
	c = (&PathConstraints{}).BlockExit(3, "west")
	path, err = m.findPathConstrained(1, 3, c)
	require.NoError(t, err)
	assert.Equal(t, []string{"east", "east"}, pathExits(path))
}

func Test_findPathConstrained_BlockedStartOrEnd(t *testing.T) {
	m := buildDetourMapper()

	_, err := m.findPathConstrained(1, 3, (&PathConstraints{}).BlockRoom(1))
	assert.ErrorIs(t, err, ErrPathBlocked, "blocked start room")

	_, err = m.findPathConstrained(1, 3, (&PathConstraints{}).BlockRoom(3))
	assert.ErrorIs(t, err, ErrPathBlocked, "blocked goal room")
}

func Test_findPathConstrained_Unreachable(t *testing.T) {
	m := buildDetourMapper()

	// Seal off both routes to room 3.
	c := (&PathConstraints{}).BlockRoom(2).BlockRoom(4).BlockRoom(5)
	_, err := m.findPathConstrained(1, 3, c)
	assert.ErrorIs(t, err, ErrPathNotFound)

	// Same via exits only.
	c = (&PathConstraints{}).BlockExit(1, "east").BlockExit(1, "south")
	_, err = m.findPathConstrained(1, 3, c)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

func Test_findPathConstrained_Cycle(t *testing.T) {
	m := buildCycleMapper()

	// Cycle with the only forward exit blocked: no route, no hang.
	c := (&PathConstraints{}).BlockExit(2, "to3")
	_, err := m.findPathConstrained(1, 3, c)
	assert.ErrorIs(t, err, ErrPathNotFound)

	// Cycle with the middle room blocked: no route, no hang.
	c = (&PathConstraints{}).BlockRoom(2)
	_, err = m.findPathConstrained(1, 3, c)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

func Test_findPathConstrained_DanglingExit(t *testing.T) {
	m := buildDetourMapper()
	// Room 1 gains an exit to a room the mapper doesn't know.
	m.crawledRooms[1].Exits["down"] = nodeExit{RoomId: 999}

	path, err := m.findPathConstrained(1, 3, nil)
	require.NoError(t, err)
	assert.Equal(t, []string{"east", "east"}, pathExits(path), "dangling exit must be skipped")

	// A graph whose only exit dangles is unreachable, not a crash.
	m2 := NewMapper(1)
	m2.crawledRooms = map[int]*mapNode{
		1: {RoomId: 1, Pos: positionDelta{}, Exits: map[string]nodeExit{"down": {RoomId: 999}}},
		2: {RoomId: 2, Pos: positionDelta{x: 1}, Exits: map[string]nodeExit{}},
	}
	_, err = m2.findPathConstrained(1, 2, nil)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

func Test_findPathConstrained_StableTieBreak(t *testing.T) {
	m := buildDiamondMapper()

	first, err := m.findPathConstrained(1, 4, nil)
	require.NoError(t, err)
	require.Equal(t, []string{"a", "c"}, pathExits(first))

	// Map iteration order is random in Go; repeat to prove the result
	// does not depend on it, with and without constraints.
	for i := 0; i < 100; i++ {
		path, err := m.findPathConstrained(1, 4, nil)
		require.NoError(t, err)
		assert.Equal(t, pathExits(first), pathExits(path))

		c := (&PathConstraints{}).BlockRoom(99) // constraint that blocks nothing
		path, err = m.findPathConstrained(1, 4, c)
		require.NoError(t, err)
		assert.Equal(t, pathExits(first), pathExits(path))
	}

	// Blocking the preferred equal-cost route flips to the other one, stably.
	c := (&PathConstraints{}).BlockRoom(2)
	for i := 0; i < 100; i++ {
		path, err := m.findPathConstrained(1, 4, c)
		require.NoError(t, err)
		assert.Equal(t, []string{"b", "d"}, pathExits(path))
	}
}

func Test_getPath_CacheIsolation(t *testing.T) {
	m := buildDetourMapper()
	pathCache.Purge()
	t.Cleanup(pathCache.Purge)

	// Poison the unconstrained cache entry for 1 => 3.
	poisoned := []pathStep{{exitName: "bogus", roomId: 99, waypoint: true}}
	pathCache.Add(pathCacheKey{startRoomId: 1, endRoomId: 3}, pathCacheValue{steps: poisoned})

	// A constrained query must not reuse the cached unconstrained route...
	c := (&PathConstraints{}).BlockRoom(2)
	path, err := getPath(m, 1, c, 3)
	require.NoError(t, err)
	assert.Equal(t, []string{"south", "east", "north"}, pathExits(path))

	// ...even when the constraints block nothing.
	path, err = getPath(m, 1, &PathConstraints{}, 3)
	require.NoError(t, err)
	assert.Equal(t, []string{"east", "east"}, pathExits(path))

	// ...and it must not write into the cache either.
	assert.Equal(t, 1, pathCache.Len(), "constrained queries must not add cache entries")

	// The unconstrained query afterwards still sees exactly the poisoned entry,
	// proving the constrained calls left the cache untouched.
	path, err = getPath(m, 1, nil, 3)
	require.NoError(t, err)
	assert.Equal(t, poisoned, path)
}

func Test_getPath_ConstrainedMultiLeg(t *testing.T) {
	m := buildDetourMapper()
	pathCache.Purge()
	t.Cleanup(pathCache.Purge)

	// 1 -> 4 -> 3 with room 2 blocked chains both legs under constraints.
	c := (&PathConstraints{}).BlockRoom(2)
	path, err := getPath(m, 1, c, 4, 3)
	require.NoError(t, err)
	assert.Equal(t, []string{"south", "east", "north"}, pathExits(path))
	assert.Equal(t, []int{4, 5, 3}, pathRooms(path))
	assert.True(t, path[len(path)-1].waypoint)

	// A leg whose room is missing from the mapper fails clearly.
	_, err = getPath(m, 1, c, 777)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

// buildGridMapper creates a width x height grid of rooms linked
// north/south/east/west. Room id = y*width + x + 1.
func buildGridMapper(width, height int) *mapper {
	m := NewMapper(1)
	m.crawledRooms = make(map[int]*mapNode, width*height)
	id := func(x, y int) int { return y*width + x + 1 }
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			node := &mapNode{
				RoomId: id(x, y),
				Pos:    positionDelta{x: x, y: y, z: 0},
				Exits:  map[string]nodeExit{},
			}
			if x > 0 {
				node.Exits["west"] = nodeExit{RoomId: id(x-1, y)}
			}
			if x < width-1 {
				node.Exits["east"] = nodeExit{RoomId: id(x+1, y)}
			}
			if y > 0 {
				node.Exits["north"] = nodeExit{RoomId: id(x, y-1)}
			}
			if y < height-1 {
				node.Exits["south"] = nodeExit{RoomId: id(x, y+1)}
			}
			m.crawledRooms[node.RoomId] = node
		}
	}
	return m
}

func Test_findPathConstrained_LargeMap(t *testing.T) {
	const width, height = 40, 40
	m := buildGridMapper(width, height)
	id := func(x, y int) int { return y*width + x + 1 }

	// Wall off column x=20 except a single gap at the bottom row.
	c := &PathConstraints{}
	for y := 0; y < height-1; y++ {
		c.BlockRoom(id(20, y))
	}

	path, err := m.findPathConstrained(id(0, 0), id(width-1, 0), c)
	require.NoError(t, err)

	// The route must detour through the gap and never enter a blocked room.
	assert.Contains(t, pathRooms(path), id(20, height-1), "route must pass through the only gap")
	for _, roomId := range pathRooms(path) {
		_, blocked := c.BlockedRooms[roomId]
		assert.False(t, blocked, "route entered blocked room %d", roomId)
	}
	// Shortest detour: 78 steps direct Manhattan distance is impossible;
	// down to the gap and back up costs 2*(height-1) extra... exactly:
	// 39 east + 39 south + 20 east... verify it is minimal by length:
	// (20 south? no) - computed expectation:
	// from (0,0) to (39,0) via (20,39): 20+39 + 19+39 = 117
	assert.Len(t, path, 117, "route should be the shortest detour through the gap")

	// Seal the wall completely: fully unreachable, and the search terminates.
	c = &PathConstraints{}
	for y := 0; y < height; y++ {
		c.BlockRoom(id(20, y))
	}
	_, err = m.findPathConstrained(id(0, 0), id(width-1, 0), c)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

func Test_findPathConstrained_SearchBound(t *testing.T) {
	// The expansion cap scales with the graph and is never hit on a healthy
	// graph, even when no route exists. A 1600-room grid would allow
	// 1600*1600+1600+1 = 2,561,601 expansions; unreachable queries must
	// finish long before that.
	const width, height = 40, 40
	m := buildGridMapper(width, height)
	delete(m.crawledRooms, width*height) // remove the goal's only neighbors' target
	goal := width*height + 1             // not in the map at all

	_, err := m.findPathConstrained(1, goal, nil)
	assert.ErrorIs(t, err, ErrRoomNotFound)

	// Disconnected goal present in the map.
	m.crawledRooms[goal] = &mapNode{RoomId: goal, Pos: positionDelta{x: 100, y: 100, z: 0}, Exits: map[string]nodeExit{}}
	_, err = m.findPathConstrained(1, goal, nil)
	assert.ErrorIs(t, err, ErrPathNotFound)
}

func Test_GetPathConstrained_StartNotFound(t *testing.T) {
	_, err := GetPathConstrained(999999, PathConstraints{}, 1)
	assert.ErrorIs(t, err, ErrPathNotFound, "expected ErrPathNotFound for missing start")
}

func Test_PathConstraints_Builders(t *testing.T) {
	c := &PathConstraints{}
	assert.False(t, c.blocksRoom(1))
	assert.False(t, c.blocksExit(1, "east"))

	c.BlockRoom(1).BlockRoom(2).BlockExit(1, "east")
	assert.True(t, c.blocksRoom(1))
	assert.True(t, c.blocksRoom(2))
	assert.False(t, c.blocksRoom(3))
	assert.True(t, c.blocksExit(1, "east"))
	assert.False(t, c.blocksExit(1, "west"))
	assert.False(t, c.blocksExit(2, "east"))

	// Nil constraints block nothing.
	var nilC *PathConstraints
	assert.False(t, nilC.blocksRoom(1))
	assert.False(t, nilC.blocksExit(1, "east"))
}

func ExampleGetPathConstrained() {
	constraints := PathConstraints{}
	constraints.BlockRoom(105).BlockExit(103, "north")
	fmt.Println("route may not enter room 105 nor take 103's north exit")
	// Output: route may not enter room 105 nor take 103's north exit
}
