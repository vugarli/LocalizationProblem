package osmprocessing

import (
	"github.com/paulmach/osm"
	"math"
)

func HaversineDistance(lat1, lon1, lat2, long2 float64) float64 {
	lat, long := lat1, lon1
	latr := lat * math.Pi / 180
	lat2r := lat2 * math.Pi / 180

	dlat := (lat2 - lat) * math.Pi / 180
	dlong := (long2 - long) * math.Pi / 180

	a := math.Sin(dlat/float64(2))*math.Sin(dlat/float64(2)) + math.Cos(latr)*math.Cos(lat2r)*math.Sin(dlong/2)*math.Sin(dlong/float64(2))
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(float64(1)-a))
	d := R * c
	return d
}

func CalculateBearing(lat1, lon1, lat2, lon2 float64) float64 {
	lat1r := lat1 * math.Pi / 180.0
	lat2r := lat2 * math.Pi / 180.0
	dlon := (lon2 - lon1) * math.Pi / 180.0

	y := math.Sin(dlon) * math.Cos(lat2r)
	x := math.Cos(lat1r)*math.Sin(lat2r) -
		math.Sin(lat1r)*math.Cos(lat2r)*math.Cos(dlon)

	bearing := math.Atan2(y, x) * 180.0 / math.Pi

	bearing = NormalizeBearing(bearing)
	return bearing
}

func NormalizeBearing(bearing float64) float64 {
	return math.Mod(bearing+360, 360)
}

func BearingDifference(bearing1, bearing2 float64) float64 {
	diff := bearing2 - bearing1

	// Normalize to [-180, 180]
	if diff > 180 {
		diff -= 360
	} else if diff < -180 {
		diff += 360
	}

	return diff
}

func DistanceToWay(lat, lon float64, way *osm.Way, nodes map[osm.NodeID]*osm.Node) float64 {
	if len(way.Nodes) < 2 {
		return math.Inf(1)
	}

	minDist := math.Inf(1)

	for i := 0; i < len(way.Nodes)-1; i++ {
		node1, ok1 := nodes[way.Nodes[i].ID]
		node2, ok2 := nodes[way.Nodes[i+1].ID]

		if !ok1 || !ok2 {
			continue
		}

		dist := DistanceToSegment(lat, lon, node1.Lat, node1.Lon, node2.Lat, node2.Lon)
		if dist < minDist {
			minDist = dist
		}
	}

	return minDist
}

func DegToRad(d float64) float64 {
	return d * math.Pi / 180.0
}

func DistanceToSegment(px, py, x1, y1, x2, y2 float64) float64 {

	px_r := DegToRad(px)
	py_r := DegToRad(py)
	x1_r := DegToRad(x1)
	y1_r := DegToRad(y1)
	x2_r := DegToRad(x2)
	y2_r := DegToRad(y2)

	dx := x2_r - x1_r
	dy := y2_r - y1_r

	if dx == 0 && dy == 0 {
		return HaversineDistance(px, py, x1, y1)
	}

	t := ((px_r-x1_r)*dx + (py_r-y1_r)*dy) / (dx*dx + dy*dy)

	t = math.Max(0, math.Min(1, t))

	closestLat := x1 + t*(x2-x1)
	closestLon := y1 + t*(y2-y1)

	return HaversineDistance(px, py, closestLat, closestLon)
}

func GetWayHeading(way *osm.Way, segmentIdx int, nodes map[osm.NodeID]*osm.Node) float64 {
	if segmentIdx < 0 || segmentIdx >= len(way.Nodes)-1 {
		return 0
	}

	node1, ok1 := nodes[way.Nodes[segmentIdx].ID]
	node2, ok2 := nodes[way.Nodes[segmentIdx+1].ID]

	if !ok1 || !ok2 {
		return 0
	}

	return CalculateBearing(node1.Lat, node1.Lon, node2.Lat, node2.Lon)
}

func GetWayHeadingAtPoint(way *osm.Way, lat, lon float64, nodes map[osm.NodeID]*osm.Node) float64 {
	if len(way.Nodes) < 2 {
		return 0
	}

	minDist := math.Inf(1)
	bestSegment := 0

	for i := 0; i < len(way.Nodes)-1; i++ {
		node1, ok1 := nodes[way.Nodes[i].ID]
		node2, ok2 := nodes[way.Nodes[i+1].ID]

		if !ok1 || !ok2 {
			continue
		}

		dist := DistanceToSegment(lat, lon, node1.Lat, node1.Lon, node2.Lat, node2.Lon)
		if dist < minDist {
			minDist = dist
			bestSegment = i
		}
	}

	return GetWayHeading(way, bestSegment, nodes)
}

func GetWayLength(way *osm.Way, nodes map[osm.NodeID]*osm.Node) float64 {
	if len(way.Nodes) < 2 {
		return 0
	}

	totalLength := 0.0

	for i := 0; i < len(way.Nodes)-1; i++ {
		node1, ok1 := nodes[way.Nodes[i].ID]
		node2, ok2 := nodes[way.Nodes[i+1].ID]

		if !ok1 || !ok2 {
			continue
		}

		totalLength += HaversineDistance(node1.Lat, node1.Lon, node2.Lat, node2.Lon)
	}

	return totalLength
}

type Bounds struct {
	MinLat, MaxLat float64
	MinLon, MaxLon float64
}

func (m *Map) CalculateBounds() Bounds {
	if len(m.Nodes) == 0 {
		return Bounds{}
	}

	bounds := Bounds{
		MinLat: math.Inf(1),
		MaxLat: math.Inf(-1),
		MinLon: math.Inf(1),
		MaxLon: math.Inf(-1),
	}

	for _, node := range m.Nodes {
		if node.Lat < bounds.MinLat {
			bounds.MinLat = node.Lat
		}
		if node.Lat > bounds.MaxLat {
			bounds.MaxLat = node.Lat
		}
		if node.Lon < bounds.MinLon {
			bounds.MinLon = node.Lon
		}
		if node.Lon > bounds.MaxLon {
			bounds.MaxLon = node.Lon
		}
	}

	return bounds
}

func (b Bounds) Contains(lat, lon float64) bool {
	return lat >= b.MinLat && lat <= b.MaxLat &&
		lon >= b.MinLon && lon <= b.MaxLon
}

func (b Bounds) GetCenter() (lat, lon float64) {
	return (b.MinLat + b.MaxLat) / 2, (b.MinLon + b.MaxLon) / 2
}
