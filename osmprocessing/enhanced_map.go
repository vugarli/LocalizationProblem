package osmprocessing

import (
	"github.com/paulmach/osm"
)

type EnhancedMap struct {
	*Map
	SpatialIndex *SpatialIndex
	WaysByID     map[osm.WayID]*osm.Way
	NodeToWays   map[osm.NodeID][]*osm.Way
	Bounds       Bounds
}

func NewEnhancedMap(m *Map) *EnhancedMap {
	em := &EnhancedMap{
		Map:        m,
		WaysByID:   make(map[osm.WayID]*osm.Way),
		NodeToWays: make(map[osm.NodeID][]*osm.Way),
	}

	em.BuildIndexes()
	return em
}

func (em *EnhancedMap) BuildIndexes() {

	for _, way := range em.Ways {
		em.WaysByID[way.ID] = way
	}

	for _, way := range em.Ways {
		for _, wn := range way.Nodes {
			em.NodeToWays[wn.ID] = append(em.NodeToWays[wn.ID], way)
		}
	}

	// 0.001 degrees = 100m cell
	em.SpatialIndex = em.Map.BuildSpatialIndex(0.001)
	em.Bounds = em.Map.CalculateBounds()
}

func (em *EnhancedMap) GetConnectedWays(nodeID osm.NodeID) []*osm.Way {
	return em.NodeToWays[nodeID]
}

func (em *EnhancedMap) IsValidPosition(lat, lon, tolerance float64) bool {
	way, dist := em.Map.FindNearestWay(lat, lon, tolerance, em.SpatialIndex)
	return way != nil && dist <= tolerance
}

func (em *EnhancedMap) FindNearestWayFast(lat, lon, maxDist float64) (*osm.Way, float64) {
	return em.Map.FindNearestWay(lat, lon, maxDist, em.SpatialIndex)
}

func (em *EnhancedMap) FindNearestNodeFast(lat, lon, maxDist float64) (*osm.Node, float64) {
	return em.Map.FindNearestNodeIndexed(lat, lon, maxDist, em.SpatialIndex)
}

func (em *EnhancedMap) GetWayHeadingAtPosition(lat, lon float64) float64 {
	way, _ := em.FindNearestWayFast(lat, lon, 50.0)
	if way == nil {
		return 0
	}
	return GetWayHeadingAtPoint(way, lat, lon, em.Nodes)
}
