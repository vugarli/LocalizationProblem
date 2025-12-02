package osmprocessing

import (
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
	m, grid := GenerateMap(1, 2, distanceWant, ToDecimalCoord(52, 19, 14, North), ToDecimalCoord(100, 43, 47, West))
	n2Lat, n2Lon := MakeCoordinateDecimal(m.Nodes[grid["0,1"]].Lat, Latitude), MakeCoordinateDecimal(m.Nodes[grid["0,1"]].Lon, Longitude)
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
	m, _ := GenerateMap(row, colum, blockSize, ToDecimalCoord(52, 19, 14, North), ToDecimalCoord(100, 43, 47, West))

	numberOfNodes := (row + 1) * (colum + 1)
	if numberOfNodes != len(m.Nodes) {
		t.Fatalf("Expected %v nodes in grid, but got %v nodes ", numberOfNodes, len(m.Nodes))
	}
}
