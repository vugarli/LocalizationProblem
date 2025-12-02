package particlefilter

import (
	"fmt"
	"math/rand"
	"roboticsproject/osmprocessing"
)

type Particle struct {
	Lat     float64
	Lon     float64
	Heading float64
	Weight  float64
}

type ParticleFilter struct {
	Particles []Particle
	Map       *osmprocessing.EnhancedMap
	rng       *rand.Rand
}

func NewParticleFilter(numberOfParticles int, EnhancedMap *osmprocessing.EnhancedMap) *ParticleFilter {
	return &ParticleFilter{
		Particles: make([]Particle, numberOfParticles),
		rng:       rand.New(rand.NewSource(123)),
		Map:       EnhancedMap,
	}
}

func (pf *ParticleFilter) InitParticles(initLat, initLon float64, errorMeter float64) {

	for i := 0; i < len(pf.Particles); i++ {

		latNoise, lonNoise := pf.rng.NormFloat64()*errorMeter/111320.0, rand.NormFloat64()*errorMeter/111320.0

		pf.Particles[i] = Particle{
			Heading: pf.rng.Float64() * 360,
			Weight:  1 / float64(len(pf.Particles)),
			Lat:     initLat + latNoise,
			Lon:     initLon + lonNoise,
		}
	}
}

func (pf *ParticleFilter) InitParticlesOnWays() {
	if len(pf.Map.Ways) == 0 {
		panic("No ways!")
	}

	// fmt.Printf("Initializing %d particles on %d roads (global initialization)...\n",
	// 		len(pf.Particles), len(pf.Map.Ways))

	particleIdx := 0
	waysProcessed := 0

	particlesPerWay := max(1, len(pf.Particles)/len(pf.Map.Ways))

	for _, way := range pf.Map.Ways {
		if len(way.Nodes) < 2 {
			continue
		}

		for p := 0; p < particlesPerWay && particleIdx < len(pf.Particles); p++ {

			segIdx := pf.rng.Intn(len(way.Nodes) - 1)
			node1 := pf.Map.Nodes[way.Nodes[segIdx].ID]
			node2 := pf.Map.Nodes[way.Nodes[segIdx+1].ID]

			if node1 == nil || node2 == nil {
				continue
			}

			t := pf.rng.Float64()
			lat := node1.Lat + t*(node2.Lat-node1.Lat)
			lon := node1.Lon + t*(node2.Lon-node1.Lon)

			heading := osmprocessing.CalculateBearing(node1.Lat, node1.Lon, node2.Lat, node2.Lon)

			if pf.rng.Float64() < 0.5 {
				heading += 180
			}
			heading += pf.rng.NormFloat64() + 5.0

			pf.Particles[particleIdx] = Particle{
				Lat:     lat,
				Lon:     lon,
				Heading: osmprocessing.NormalizeBearing(heading),
				Weight:  1.0 / float64(len(pf.Particles)),
			}
			particleIdx++
		}
		waysProcessed++
	}

	for particleIdx < len(pf.Particles) {
		way := pf.Map.Ways[pf.rng.Intn(len(pf.Map.Ways))]
		if len(way.Nodes) < 2 {
			continue
		}

		segIdx := pf.rng.Intn(len(way.Nodes) - 1)
		node1 := pf.Map.Nodes[way.Nodes[segIdx].ID]
		node2 := pf.Map.Nodes[way.Nodes[segIdx+1].ID]

		if node1 == nil || node2 == nil {
			continue
		}

		t := pf.rng.Float64()
		lat := node1.Lat + t*(node2.Lat-node1.Lat)
		lon := node1.Lon + t*(node2.Lon-node1.Lon)
		heading := osmprocessing.CalculateBearing(node1.Lat, node1.Lon, node2.Lat, node2.Lon)

		if pf.rng.Float64() < 0.5 {
			heading += 180
		}

		pf.Particles[particleIdx] = Particle{
			Lat:     lat,
			Lon:     lon,
			Heading: osmprocessing.NormalizeBearing(heading),
			Weight:  1.0 / float64(len(pf.Particles)),
		}
		particleIdx++
	}

	fmt.Printf("Initialized %d particles across %d roads\n", particleIdx, waysProcessed)
}
