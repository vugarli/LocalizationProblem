package osmprocessing

import (
	"fmt"
	"math"
	"testing"

	"github.com/paulmach/osm"
)

func assertWithinPercent(t *testing.T, got, want, percentTolerance float64) {
	t.Helper()

	if want == 0 {
		if math.Abs(got) > percentTolerance {
			t.Errorf("got %.6f, want 0 (absolute diff: %.6f > %.6f)",
				got, math.Abs(got), percentTolerance)
		}
		return
	}

	relativeError := math.Abs(got-want) / math.Abs(want)

	if relativeError > percentTolerance/100.0 {
		t.Errorf("got %.6f, want %.6f (deviation: %.2f%%, max: %.2f%%)",
			got, want, relativeError*100, percentTolerance)
	}
}

func TestHaversineDistance(t *testing.T) {
	tests := []struct {
		name     string
		lat1     float64
		lon1     float64
		lat2     float64
		lon2     float64
		expected float64
	}{
		{
			name: "1 degree north from equator",
			lat1: 0, lon1: 0,
			lat2: 1, lon2: 0,
			expected: 111320.0,
		},
		{
			name: "1 degree east from equator",
			lat1: 0, lon1: 0,
			lat2: 0, lon2: 1,
			expected: 111320.0,
		},
		{
			name: "zero distance",
			lat1: 46.0, lon1: 7.0,
			lat2: 46.0, lon2: 7.0,
			expected: 0.0,
		},
		{
			name: "100km approximately",
			lat1: 46.0, lon1: 7.0,
			lat2: 46.898, lon2: 7.0,
			expected: 100000.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := HaversineDistance(tt.lat1, tt.lon1, tt.lat2, tt.lon2)
			assertWithinPercent(t, got, tt.expected, 0.5)
		})
	}
}

func TestCalculateBearing(t *testing.T) {
	tests := []struct {
		name     string
		lat1     float64
		lon1     float64
		lat2     float64
		lon2     float64
		expected float64
	}{
		{
			name: "due north",
			lat1: 0, lon1: 0,
			lat2: 1, lon2: 0,
			expected: 0,
		},
		{
			name: "due east",
			lat1: 0, lon1: 0,
			lat2: 0, lon2: 1,
			expected: 90,
		},
		{
			name: "due south",
			lat1: 1, lon1: 0,
			lat2: 0, lon2: 0,
			expected: 180,
		},
		{
			name: "due west",
			lat1: 0, lon1: 1,
			lat2: 0, lon2: 0,
			expected: 270,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := CalculateBearing(tt.lat1, tt.lon1, tt.lat2, tt.lon2)
			assertWithinPercent(t, got, tt.expected, 1.0) // 1% tolerance for bearings
		})
	}
}

func TestBearingDifference(t *testing.T) {
	tests := []struct {
		name     string
		bearing1 float64
		bearing2 float64
		expected float64
	}{
		{"same direction", 90, 90, 0},
		{"small clockwise", 90, 100, 10},
		{"small counter-clockwise", 100, 90, -10},
		{"wrap around 0", 350, 10, 20},
		{"wrap around 360", 10, 350, -20},
		{"opposite directions", 0, 180, 180},
		{"opposite directions reverse", 180, 0, -180},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := BearingDifference(tt.bearing1, tt.bearing2)
			if math.Abs(got-tt.expected) > 0.1 {
				t.Errorf("got %.2f, want %.2f", got, tt.expected)
			}
		})
	}
}

func TestDistanceToWay(t *testing.T) {

	m, grid := generateMap(0, 1, 100,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateHorizontalWays(m, grid, 0, 1)

	way := m.Ways[0]

	tests := []struct {
		name    string
		lat     float64
		lon     float64
		maxDist float64
	}{
		{
			name:    "point on road",
			lat:     m.Nodes[grid["0,0"]].Lat,
			lon:     m.Nodes[grid["0,0"]].Lon,
			maxDist: 1.0, // Should be ~0m
		},
		{
			name:    "point 50m north of road",
			lat:     m.Nodes[grid["0,0"]].Lat + 0.00045, // ~50m north
			lon:     m.Nodes[grid["0,0"]].Lon,
			maxDist: 60.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			dist := DistanceToWay(tt.lat, tt.lon, way, m.Nodes)
			if dist > tt.maxDist {
				t.Errorf("distance %.2fm exceeds max %.2fm", dist, tt.maxDist)
			}
		})
	}
}

func TestGetWayLength(t *testing.T) {
	blockSize := 250.0
	m, grid := generateMap(0, 1, blockSize,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateHorizontalWays(m, grid, 0, 1)
	way := m.Ways[0]

	length := GetWayLength(way, m.Nodes)
	assertWithinPercent(t, length, blockSize, 1.2)
}

func TestGetWayHeading(t *testing.T) {
	m, grid := generateMap(0, 1, 100,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateHorizontalWays(m, grid, 0, 1)
	heading := GetWayHeading(m.Ways[0], 0, m.Nodes)

	assertWithinPercent(t, heading, 90.0, 5.0)
}

func TestSpatialIndex(t *testing.T) {
	m, _ := generateMap(3, 3, 100,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	index := m.BuildSpatialIndex(0.001)

	t.Run("query ways in radius", func(t *testing.T) {
		bounds := m.CalculateBounds()
		centerLat, centerLon := bounds.GetCenter()

		ways := index.QueryWays(centerLat, centerLon, 500.0)

		if len(ways) == 0 {
			t.Error("expected to find ways near center")
		}
	})

	t.Run("query nodes in radius", func(t *testing.T) {
		bounds := m.CalculateBounds()
		centerLat, centerLon := bounds.GetCenter()

		nodes := index.QueryNodes(centerLat, centerLon, 500.0)

		if len(nodes) == 0 {
			t.Error("expected to find nodes near center")
		}
	})

	t.Run("query empty area", func(t *testing.T) {
		ways := index.QueryWays(0, 0, 10.0)
		if len(ways) > 0 {
			t.Error("expected no ways in empty area")
		}
	})
}

func TestFindNearestWayWithIndex(t *testing.T) {
	m, grid := generateMap(2, 2, 200,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateAllWays(m, grid, 2, 2)
	index := m.BuildSpatialIndex(0.001)

	originNode := m.Nodes[grid["0,0"]]

	way, dist := m.FindNearestWay(originNode.Lat, originNode.Lon, 100.0, index)

	if way == nil {
		t.Fatal("expected to find nearest way")
	}

	if dist > 10.0 {
		t.Errorf("distance %.2fm too large for point on road", dist)
	}
}

func TestEnhancedMap(t *testing.T) {
	m, grid := generateMap(2, 2, 150,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateAllWays(m, grid, 2, 2)
	em := NewEnhancedMap(m)

	t.Run("builds indexes", func(t *testing.T) {
		if em.SpatialIndex == nil {
			t.Error("spatial index not built")
		}
		if len(em.WaysByID) != len(m.Ways) {
			t.Errorf("WaysByID has %d entries, expected %d", len(em.WaysByID), len(m.Ways))
		}
	})

	t.Run("calculates bounds", func(t *testing.T) {
		if em.Bounds.MinLat == math.Inf(1) {
			t.Error("bounds not calculated")
		}

		centerLat, centerLon := em.Bounds.GetCenter()
		if !em.Bounds.Contains(centerLat, centerLon) {
			t.Error("center should be within bounds")
		}
	})

	t.Run("finds connected ways", func(t *testing.T) {

		centerNodeID := grid["1,1"]
		connectedWays := em.GetConnectedWays(centerNodeID)

		if len(connectedWays) < 2 {
			t.Errorf("center node should have multiple connections, got %d", len(connectedWays))
		}
	})

	t.Run("validates positions", func(t *testing.T) {

		node := m.Nodes[grid["0,0"]]
		if !em.IsValidPosition(node.Lat, node.Lon, 10.0) {
			t.Error("node position should be valid")
		}

		if em.IsValidPosition(0, 0, 10.0) {
			t.Error("distant position should be invalid")
		}
	})
}

func TestCalculateBounds(t *testing.T) {
	m, grid := generateMap(2, 2, 200,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	bounds := m.CalculateBounds()

	t.Run("bounds contain all nodes", func(t *testing.T) {
		for key, nodeID := range grid {
			node := m.Nodes[nodeID]
			if !bounds.Contains(node.Lat, node.Lon) {
				t.Errorf("node %s not within bounds", key)
			}
		}
	})

	t.Run("center is valid", func(t *testing.T) {
		centerLat, centerLon := bounds.GetCenter()
		if !bounds.Contains(centerLat, centerLon) {
			t.Error("center should be within bounds")
		}
	})
}

func generateHorizontalWays(m *Map, grid map[string]osm.NodeID, row, cols int) []*osm.Way {
	ways := make([]*osm.Way, 0)
	wayID := osm.WayID(1)

	for col := 0; col < cols; col++ {
		n1 := grid[fmt.Sprintf("%d,%d", row, col)]
		n2 := grid[fmt.Sprintf("%d,%d", row, col+1)]

		way := &osm.Way{
			ID: wayID,
			Nodes: []osm.WayNode{
				{ID: n1},
				{ID: n2},
			},
		}
		ways = append(ways, way)
		wayID++
	}

	return ways
}

func generateAllWays(m *Map, grid map[string]osm.NodeID, rows, cols int) []*osm.Way {
	ways := make([]*osm.Way, 0)
	wayID := osm.WayID(1)

	// horizontal ways
	for row := 0; row <= rows; row++ {
		for col := 0; col < cols; col++ {
			n1 := grid[fmt.Sprintf("%d,%d", row, col)]
			n2 := grid[fmt.Sprintf("%d,%d", row, col+1)]

			way := &osm.Way{
				ID: wayID,
				Nodes: []osm.WayNode{
					{ID: n1},
					{ID: n2},
				},
			}
			ways = append(ways, way)
			wayID++
		}
	}

	// vertical ways
	for col := 0; col <= cols; col++ {
		for row := 0; row < rows; row++ {
			n1 := grid[fmt.Sprintf("%d,%d", row, col)]
			n2 := grid[fmt.Sprintf("%d,%d", row+1, col)]

			way := &osm.Way{
				ID: wayID,
				Nodes: []osm.WayNode{
					{ID: n1},
					{ID: n2},
				},
			}
			ways = append(ways, way)
			wayID++
		}
	}

	return ways
}

func BenchmarkFindNearestWay(b *testing.B) {
	m, grid := generateMap(10, 10, 100,
		ToDecimalCoord(46, 0, 0, North),
		ToDecimalCoord(7, 0, 0, East))

	m.Ways = generateAllWays(m, grid, 10, 10)
	index := m.BuildSpatialIndex(0.001)

	testNode := m.Nodes[grid["5,5"]]

	b.Run("with spatial index", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			m.FindNearestWay(testNode.Lat, testNode.Lon, 100.0, index)
		}
	})

	b.Run("without spatial index", func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			m.FindNearestWay(testNode.Lat, testNode.Lon, 100.0, nil)
		}
	})
}
