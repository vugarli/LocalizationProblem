package osmprocessing

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"os"
	"sort"

	"github.com/grab/gosm"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

type Map struct {
	Ways  []*osm.Way
	Nodes map[osm.NodeID]*osm.Node
}

var drivableHighways = map[string]bool{
	"motorway":       true,
	"motorway_link":  true,
	"trunk":          true,
	"trunk_link":     true,
	"primary":        true,
	"primary_link":   true,
	"secondary":      true,
	"secondary_link": true,
	"tertiary":       true,
	"tertiary_link":  true,
	"living_street":  true,
	"residential":    true,
	"service":        true,
	"unclassified":   true,
	"track":          true,
}

func CalculateDestinationPoint(latOrgn, longOrgn CoordinateDecimal, bearing BearingDecimal,
	distance float64) (latDest, longDest CoordinateDecimal) {

	latOrgRad := float64(latOrgn.DecimalDegree) * float64(math.Pi) / 180.0
	longOrgRad := float64(longOrgn.DecimalDegree) * float64(math.Pi) / 180.0
	bearingRad := float64(bearing) * float64(math.Pi) / 180.0

	// φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
	// λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )
	latDestRad := math.Asin(math.Sin(latOrgRad)*math.Cos(distance/R) + math.Cos(latOrgRad)*
		math.Sin(distance/R)*math.Cos(float64(bearingRad)))

	longDestRad := longOrgRad + math.Atan2(math.Sin(float64(bearingRad))*math.Sin(distance/R)*math.Cos(latOrgRad),
		math.Cos(distance/R)-math.Sin(latOrgRad)*math.Sin(latDestRad))

	latDestDeg := latDestRad * 180 / float64(math.Pi)
	longDestDeg := longDestRad * 180 / float64(math.Pi)

	latDestDeg = math.Round(latDestDeg*1e4) / 1e4
	longDestDeg = math.Round(longDestDeg*1e4) / 1e4

	latDir := North
	if latDestDeg < 0 {
		latDir = South
	}

	longDir := East
	if longDestDeg < 0 {
		longDir = West
	}

	lat := MakeCoordinateDecimal(latDestDeg, latDir)
	long := MakeCoordinateDecimal(longDestDeg, longDir)

	return lat, long
}

func DistanceTo(n *osm.Node, lat2, long2 CoordinateDecimal) float64 {
	lat, long := n.Lat, n.Lon
	latr := lat * math.Pi / 180
	lat2r := lat2.DecimalDegree * math.Pi / 180

	dlat := (lat2.DecimalDegree - lat) * math.Pi / 180
	dlong := (long2.DecimalDegree - long) * math.Pi / 180

	a := math.Sin(dlat/float64(2))*math.Sin(dlat/float64(2)) + math.Cos(latr)*math.Cos(lat2r)*math.Sin(dlong/2)*math.Sin(dlong/float64(2))
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(float64(1)-a))
	d := R * c
	return d
}

func (m *Map) FindNearestNode(orgLat, orgLong CoordinateDecimal) (*osm.Node, float64) {
	if len(m.Nodes) == 0 {
		return nil, 0
	}

	minDistance := math.Inf(1)
	var nearestNode *osm.Node

	for _, node := range m.Nodes {
		dist := DistanceTo(node, orgLat, orgLong)
		if dist < minDistance {
			minDistance = dist
			nearestNode = node
		}
	}

	return nearestNode, minDistance
}

func myMapF[T, V any](ts []T, fn func(T) V) []V {
	result := make([]V, len(ts))
	for i, t := range ts {
		result[i] = fn(t)
	}
	return result
}

func ToGosmNode(n *osm.Node) *gosm.Node {
	return &gosm.Node{
		Latitude:  n.Lat,
		Longitude: n.Lon,
		ID:        int64(n.ID),
	}
}
func ToGosmWay(w *osm.Way) *gosm.Way {
	return &gosm.Way{
		ID:      int64(w.ID),
		Tags:    w.TagMap(),
		NodeIDs: myMapF(w.Nodes.NodeIDs(), func(id osm.NodeID) int64 { return int64(id) }),
	}
}

func SaveMapToOSM(fmap *Map, fname string) error {
	f, err := os.Create(fmt.Sprintf("%s.osm.pbf", fname))
	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	encoder := gosm.NewEncoder(&gosm.NewEncoderRequiredInput{
		RequiredFeatures: []string{"OsmSchema-V0.6", "DenseNodes"},
		Writer:           f,
	},
		gosm.WithWritingProgram("wp1"),
		gosm.WithZlipEnabled(true),
	)

	defer encoder.Close()

	errChan, err := encoder.Start()
	if err != nil {
		return err
	}

	var errs []error
	go func() {
		for e := range errChan {
			errs = append(errs, e)
		}
	}()

	sorts := make([]*osm.Node, len(fmap.Nodes))
	i := 0
	for _, node := range fmap.Nodes {
		sorts[i] = node
		i++
	}

	sort.Slice(sorts, func(i, j int) bool {
		return sorts[i].ID < sorts[j].ID
	})

	gosmWriteElementsMax := 8000

	nodes := make([]*gosm.Node, 0, gosmWriteElementsMax)
	count := 0
	for _, n := range sorts {
		nodes = append(nodes, ToGosmNode(n))
		count++
		if count == gosmWriteElementsMax {
			encoder.AppendNodes(nodes)
			nodes = make([]*gosm.Node, 0, gosmWriteElementsMax)
			count = 0
		}
	}
	if len(nodes) > 0 {
		encoder.AppendNodes(nodes)
	}

	encoder.Flush(gosm.NodeType)

	sortsw := make([]*osm.Way, len(fmap.Ways))
	i = 0
	for _, way := range fmap.Ways {
		sortsw[i] = way
		i++
	}
	sort.Slice(sortsw, func(i, j int) bool {
		return sortsw[i].ID < sortsw[j].ID
	})

	ways := make([]*gosm.Way, 0, gosmWriteElementsMax)
	count = 0
	for _, w := range sortsw {
		ways = append(ways, ToGosmWay(w))
		count++
		if count == gosmWriteElementsMax {
			encoder.AppendWays(ways)
			ways = make([]*gosm.Way, 0, gosmWriteElementsMax)
			count = 0
		}
	}
	if len(ways) > 0 {
		encoder.AppendWays(ways)
	}
	encoder.Flush(gosm.WayType)

	if len(errs) > 0 {
		fmt.Print(errs)
	}

	return nil
}

func ExtractObjects(fname string, save bool) *Map {

	f, err := os.Open(fname)
	if err != nil {
		panic(err)
	}
	defer f.Close()

	nodes := make(map[osm.NodeID]*osm.Node)
	ways := []*osm.Way{}

	scanner := osmpbf.New(context.Background(), f, 4)

	for scanner.Scan() {
		obj := scanner.Object()

		switch o := obj.(type) {

		case *osm.Node:
			nodes[o.ID] = o

		case *osm.Way:
			if o.Tags == nil {
				continue
			}
			if o.Tags.HasTag("building") {
				continue
			}
			if ok := o.Tags.HasTag("highway"); ok {
				hw := o.Tags.FindTag("highway").Value
				if drivableHighways[hw] {
					ways = append(ways, o)
				}
			}
		}
	}

	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}

	splitWays := splitAtIntersections(ways)

	usedNodes := make(map[osm.NodeID]bool)

	for _, w := range ways {
		for _, nid := range w.Nodes.NodeIDs() {
			usedNodes[nid] = true
		}
	}

	filteredNodes := make(map[osm.NodeID]*osm.Node)
	for nid := range usedNodes {
		if n, ok := nodes[nid]; ok {
			filteredNodes[nid] = n
		}
	}

	out := Map{
		Ways:  splitWays,
		Nodes: filteredNodes,
	}
	return &out
}

func (out *Map) SaveObjects(fname string) (string, error) {
	j, err := json.MarshalIndent(out, "", "  ")
	if err != nil {
		return "", fmt.Errorf("failed to read %q %w", fname, err)
	}

	outputFileName := fmt.Sprintf("%s.json", fname)
	os.WriteFile(outputFileName, j, 0666)

	if err := os.WriteFile(outputFileName, j, 0666); err != nil {
		return "", fmt.Errorf("failed to write file %q %w", outputFileName, err)
	}

	fmt.Println("Done:", outputFileName)
	return outputFileName, nil
}

func (m *Map) LoadObjects(fname string) error {
	data, err := os.ReadFile(fname)
	if err != nil {
		return fmt.Errorf("failed read %q %w", fname, err)
	}
	if err := json.Unmarshal(data, m); err != nil {
		return fmt.Errorf("failed to unmarshal %q %w", fname, err)
	}

	return nil
}

func splitAtIntersections(ways []*osm.Way) []*osm.Way {

	count := make(map[osm.NodeID]int)
	for _, w := range ways {
		for _, n := range w.Nodes {
			count[n.ID]++
		}
	}

	var out []*osm.Way
	var nextID int64 = 1

	for _, w := range ways {

		current := &osm.Way{
			ID:    osm.WayID(nextID),
			Tags:  append(osm.Tags(nil), w.Tags...),
			Nodes: make([]osm.WayNode, 0, len(w.Nodes)),
		}
		nextID++

		for _, n := range w.Nodes {
			current.Nodes = append(current.Nodes, n)

			if count[n.ID] > 1 && len(current.Nodes) > 1 {

				seg := &osm.Way{
					ID:    current.ID,
					Tags:  append(osm.Tags(nil), current.Tags...),
					Nodes: append([]osm.WayNode(nil), current.Nodes...),
				}
				out = append(out, seg)

				current = &osm.Way{
					ID:    osm.WayID(nextID),
					Tags:  append(osm.Tags(nil), w.Tags...),
					Nodes: []osm.WayNode{n},
				}
				nextID++
			}
		}

		if len(current.Nodes) > 1 {
			seg := &osm.Way{
				ID:    current.ID,
				Tags:  append(osm.Tags(nil), current.Tags...),
				Nodes: append([]osm.WayNode(nil), current.Nodes...),
			}
			out = append(out, seg)
		}
	}

	return out
}

// func GenerateAllWays(m *Map, grid map[string]osm.NodeID, rows, cols int) []*osm.Way {
// 	ways := make([]*osm.Way, 0)
// 	wayID := osm.WayID(1)

// 	// horizontal ways
// 	for row := 0; row <= rows; row++ {
// 		for col := 0; col < cols; col++ {
// 			n1 := grid[fmt.Sprintf("%d,%d", row, col)]
// 			n2 := grid[fmt.Sprintf("%d,%d", row, col+1)]

// 			way := &osm.Way{
// 				ID: wayID,
// 				Nodes: []osm.WayNode{
// 					{ID: n1},
// 					{ID: n2},
// 				},
// 			}
// 			ways = append(ways, way)
// 			wayID++
// 		}
// 	}

// 	// vertical ways
// 	for col := 0; col <= cols; col++ {
// 		for row := 0; row < rows; row++ {
// 			n1 := grid[fmt.Sprintf("%d,%d", row, col)]
// 			n2 := grid[fmt.Sprintf("%d,%d", row+1, col)]

// 			way := &osm.Way{
// 				ID: wayID,
// 				Nodes: []osm.WayNode{
// 					{ID: n1},
// 					{ID: n2},
// 				},
// 			}
// 			ways = append(ways, way)
// 			wayID++
// 		}
// 	}

//		return ways
//	}
func GenerateMap(row, column int, blockSize float64, latOrg, longOrg CoordinateDecimal) (*Map, map[string]osm.NodeID) {
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
