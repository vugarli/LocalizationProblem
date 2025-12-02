package osmprocessing

import (
	"fmt"
	"github.com/paulmach/osm"
	"math"
	"testing"
)

var Id int64

func nextID(t *testing.T) (Id int64) {
	t.Helper()
	Id++
	return
}

func TestDestinationPointFromSrcGivenLatLongBearing(t *testing.T) {

	cases := []struct {
		LatOrgn  CoordinateDecimal
		LongOrgn CoordinateDecimal
		Bearing  BearingDecimal
		Distance float64
		LatDest  CoordinateDecimal
		LongDest CoordinateDecimal
	}{
		{ToDecimalCoord(53, 19, 14, North), ToDecimalCoord(1, 43, 47, West),
			ToDecimalBearing(96, 1, 18), 124.8e3, ToDecimalCoord(53, 11, 18, North), ToDecimalCoord(0, 8, 0, East)},

		{ToDecimalCoord(52, 19, 14, North), ToDecimalCoord(100, 43, 47, West),
			ToDecimalBearing(31, 1, 18), 93.8e3, ToDecimalCoord(53, 2, 29, North), ToDecimalCoord(100, 0, 24, West)},
	}

	for _, c := range cases {
		t.Run("test", func(t *testing.T) {
			gotlat, gotlong := CalculateDestinationPoint(c.LatOrgn, c.LongOrgn, c.Bearing, c.Distance)
			if !coordsEqual(gotlat, c.LatDest) || !coordsEqual(gotlong, c.LongDest) {
				t.Fatalf("Got %v,%v but wanted %v,%v", gotlat, gotlong, c.LatDest, c.LongDest)
			}
		})
	}
}

func TestDistanceTo(t *testing.T) {
	distanceWant := 5000.0
	m, grid := generateMap(1, 2, distanceWant, ToDecimalCoord(52, 19, 14, North), ToDecimalCoord(100, 43, 47, West))
	n2Lat, n2Lon := makeCoordinateDecimal(m.Nodes[grid["0,1"]].Lat, Latitude), makeCoordinateDecimal(m.Nodes[grid["0,1"]].Lon, Longitude)
	n := m.Nodes[grid["0,0"]]

	distanceGot := DistanceTo(n, n2Lat, n2Lon)
	relativeError := math.Abs(distanceGot-distanceWant) / distanceWant
	allowedError := 0.003

	if relativeError > allowedError {
		t.Fatalf("Expected %v but got %v", distanceWant, distanceGot)
	}
}

func coordsEqual(a, b CoordinateDecimal) bool {
	return math.Abs(a.DecimalDegree-b.DecimalDegree) < 0.0001
}

func TestGenerateMap_Simple(t *testing.T) {
	row, colum := 3, 3
	blockSize := 100.0
	m, _ := generateMap(row, colum, blockSize, ToDecimalCoord(52, 19, 14, North), ToDecimalCoord(100, 43, 47, West))

	numberOfNodes := (row + 1) * (colum + 1)
	if numberOfNodes != len(m.Nodes) {
		t.Fatalf("Expected %v nodes in grid, but got %v nodes ", numberOfNodes, len(m.Nodes))
	}
}

// func TestFindNearestNode(t *testing.T) {
// 	myMap, grid := generateMap(3, 3, 500, ToDecimalCoord(53, 19, 14, North), ToDecimalCoord(1, 43, 47, West))
// 	// go southern-east (100 deg) 100 meters. nearest node would be 0,1
// 	orgLat, orgLong := CalculateDestinationPoint(ToDecimalCoord(53, 19, 14, North), ToDecimalCoord(1, 43, 47, West),
// 		BearingDecimal(100), 100)

// 	wantNodeId := grid["0,1"]
// 	wantLat := makeCoordinateDecimal(myMap.Nodes[wantNodeId].Lat, Latitude)
// 	wantLong := makeCoordinateDecimal(myMap.Nodes[wantNodeId].Lon, Longitude)

// 	gotNode := myMap.FindNearestNode(orgLat, orgLong)

// 	if n, ok := myMap.Nodes[gotNode.ID]; ok {
// 		if !(coordsEqual(wantLat, makeCoordinateDecimal(n.Lat, Latitude)) && coordsEqual(wantLong, makeCoordinateDecimal(n.Lon, Longitude))) {
// 			t.Fatalf("Expected (%v,%v), but got (%v,%v)", wantLat, wantLong, makeCoordinateDecimal(n.Lat, Latitude), makeCoordinateDecimal(n.Lon, Longitude))
// 		}
// 	} else {
// 		t.Fatalf("Expected node (%v) not found!", gotNode.ID)
// 	}
// }

func generateMap(row, column int, blockSize float64, latOrg, longOrg CoordinateDecimal) (*Map, map[string]osm.NodeID) {
	m := &Map{
		Nodes: make(map[osm.NodeID]*osm.Node),
		Ways:  make([]*osm.Way, 0),
	}

	grid := make(map[string]osm.NodeID)
	var nodeID osm.NodeID = 0

	for r := 0; r <= row; r++ {
		for c := 0; c <= column; c++ {

			nlat, nlong := CalculateDestinationPoint(latOrg, longOrg, BearingDecimal(0), blockSize*float64(r))

			elat, elong := CalculateDestinationPoint(nlat, nlong, BearingDecimal(90), blockSize*float64(c))

			m.Nodes[nodeID] = &osm.Node{
				ID:  nodeID,
				Lat: elat.DecimalDegree,
				Lon: elong.DecimalDegree,
			}

			grid[fmt.Sprintf("%d,%d", r, c)] = nodeID
			nodeID++
		}
	}

	var wayID osm.WayID = 1

	for r := 0; r <= row; r++ {
		for c := 0; c < column; c++ { // Note: c < column (not <=)
			node1ID := grid[fmt.Sprintf("%d,%d", r, c)]
			node2ID := grid[fmt.Sprintf("%d,%d", r, c+1)]

			way := &osm.Way{
				ID: wayID,
				Tags: osm.Tags{
					{Key: "highway", Value: "residential"},
					{Key: "name", Value: fmt.Sprintf("Street %d", r)},
				},
				Nodes: []osm.WayNode{
					{ID: node1ID},
					{ID: node2ID},
				},
			}

			m.Ways = append(m.Ways, way)
			wayID++
		}
	}

	for c := 0; c <= column; c++ {
		for r := 0; r < row; r++ { // Note: r < row (not <=)
			node1ID := grid[fmt.Sprintf("%d,%d", r, c)]
			node2ID := grid[fmt.Sprintf("%d,%d", r+1, c)]

			way := &osm.Way{
				ID: wayID,
				Tags: osm.Tags{
					{Key: "highway", Value: "residential"},
					{Key: "name", Value: fmt.Sprintf("Avenue %d", c)},
				},
				Nodes: []osm.WayNode{
					{ID: node1ID},
					{ID: node2ID},
				},
			}

			m.Ways = append(m.Ways, way)
			wayID++
		}
	}

	return m, grid
}
