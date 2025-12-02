package osmprocessing

import (
	"github.com/paulmach/osm"
	"math"
)

type GridCell struct {
	LatIdx, LonIdx int
}

type SpatialIndex struct {
	wayGrid  map[GridCell][]*osm.Way
	nodeGrid map[GridCell][]*osm.Node
	cellSize float64 // in degrees
}

func NewSpatialIndex(cellSize float64) *SpatialIndex {
	return &SpatialIndex{
		wayGrid:  make(map[GridCell][]*osm.Way),
		nodeGrid: make(map[GridCell][]*osm.Node),
		cellSize: cellSize,
	}
}

func (si *SpatialIndex) getCell(lat, lon float64) GridCell {
	return GridCell{
		LatIdx: int(math.Floor(lat / si.cellSize)),
		LonIdx: int(math.Floor(lon / si.cellSize)),
	}
}

func (si *SpatialIndex) InsertWay(way *osm.Way, nodes map[osm.NodeID]*osm.Node) {
	seen := make(map[GridCell]bool)
	for _, wn := range way.Nodes {
		if node, ok := nodes[wn.ID]; ok {
			cell := si.getCell(node.Lat, node.Lon)
			if !seen[cell] {
				si.wayGrid[cell] = append(si.wayGrid[cell], way)
				seen[cell] = true
			}
		}
	}
}

func (si *SpatialIndex) InsertNode(node *osm.Node) {
	cell := si.getCell(node.Lat, node.Lon)
	si.nodeGrid[cell] = append(si.nodeGrid[cell], node)
}

func (si *SpatialIndex) QueryWays(lat, lon, radius float64) []*osm.Way {
	centerCell := si.getCell(lat, lon)

	radiusInDegrees := radius / 111000.0 // 1 degree = 111km
	radiusInCells := int(math.Ceil(radiusInDegrees/si.cellSize)) + 1

	seen := make(map[osm.WayID]bool)
	var results []*osm.Way

	for dLat := -radiusInCells; dLat <= radiusInCells; dLat++ {
		for dLon := -radiusInCells; dLon <= radiusInCells; dLon++ {
			cell := GridCell{
				LatIdx: centerCell.LatIdx + dLat,
				LonIdx: centerCell.LonIdx + dLon,
			}

			for _, way := range si.wayGrid[cell] {
				if !seen[way.ID] {
					results = append(results, way)
					seen[way.ID] = true
				}
			}
		}
	}

	return results
}

func (si *SpatialIndex) QueryNodes(lat, lon, radius float64) []*osm.Node {
	centerCell := si.getCell(lat, lon)

	radiusInDegrees := radius / 111000.0
	radiusInCells := int(math.Ceil(radiusInDegrees/si.cellSize)) + 1

	seen := make(map[osm.NodeID]bool)
	var results []*osm.Node

	for dLat := -radiusInCells; dLat <= radiusInCells; dLat++ {
		for dLon := -radiusInCells; dLon <= radiusInCells; dLon++ {
			cell := GridCell{
				LatIdx: centerCell.LatIdx + dLat,
				LonIdx: centerCell.LonIdx + dLon,
			}

			for _, node := range si.nodeGrid[cell] {
				if !seen[node.ID] {
					results = append(results, node)
					seen[node.ID] = true
				}
			}
		}
	}

	return results
}

func (m *Map) BuildSpatialIndex(cellSize float64) *SpatialIndex {
	index := NewSpatialIndex(cellSize)

	for _, way := range m.Ways {
		index.InsertWay(way, m.Nodes)
	}

	for _, node := range m.Nodes {
		index.InsertNode(node)
	}

	return index
}

func (m *Map) FindNearestWay(lat, lon, maxDist float64, index *SpatialIndex) (*osm.Way, float64) {
	var candidates []*osm.Way

	if index != nil {
		candidates = index.QueryWays(lat, lon, maxDist)
	} else {
		candidates = m.Ways
	}

	var nearest *osm.Way
	minDist := math.Inf(1)

	for _, way := range candidates {
		dist := DistanceToWay(lat, lon, way, m.Nodes)
		if dist < minDist && dist <= maxDist {
			minDist = dist
			nearest = way
		}
	}

	return nearest, minDist
}

func (m *Map) FindNearestNodeIndexed(lat, lon, maxDist float64, index *SpatialIndex) (*osm.Node, float64) {
	var candidates []*osm.Node

	if index != nil {
		candidates = index.QueryNodes(lat, lon, maxDist)
	} else {
		for _, node := range m.Nodes {
			candidates = append(candidates, node)
		}
	}

	if len(candidates) == 0 {
		return nil, 0
	}

	minDist := math.Inf(1)
	var nearest *osm.Node

	for _, node := range candidates {
		dist := HaversineDistance(lat, lon, node.Lat, node.Lon)
		if dist < minDist && dist <= maxDist {
			minDist = dist
			nearest = node
		}
	}

	return nearest, minDist
}
