package particlefilter

import (
	"fmt"
	"math"
	"math/rand"
	"roboticsproject/osmprocessing"
)

type VOReading struct {
	Distance, Angle float64
}

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

func gaussianProbability(mean float64, sigma float64, x float64) float64 {
	//	P(x) = (1 / √(2πσ²)) × exp(-(x-μ)² / (2σ²))
	exponent := -((x - mean) * (x - mean)) / (2 * sigma * sigma)
	return math.Exp(exponent)
}

func (pf *ParticleFilter) gaussianNoise(mean, stdDev float64) float64 {
	u1 := pf.rng.Float64()
	u2 := pf.rng.Float64()

	for u1 == 0 {
		u1 = pf.rng.Float64()
	}

	z := math.Sqrt(-2*math.Log(u1)) * math.Cos(2*math.Pi*u2)

	return mean + stdDev*z
}

func (pf *ParticleFilter) ParticleUpdateWeigh() {

	totalWeigh := 0.0

	for i, particle := range pf.Particles {

		nearestWay, distance := pf.Map.FindNearestWayFast(particle.Lat, particle.Lon, 50.0)

		if nearestWay == nil {
			pf.Particles[i].Weight = 0.001
			totalWeigh += pf.Particles[i].Weight
			continue
		}

		probabilityBasedOnDistance := gaussianProbability(0, 2.0, distance)

		wayBearing := osmprocessing.GetWayHeadingAtPoint(nearestWay, pf.Particles[i].Lat, pf.Particles[i].Lon, pf.Map.Nodes)
		bearingDiff := math.Abs(osmprocessing.BearingDifference(particle.Heading, wayBearing))
		probabilityBasedOnBearing := gaussianProbability(0, 15, bearingDiff)

		pf.Particles[i].Weight = probabilityBasedOnBearing * probabilityBasedOnDistance

		if pf.Particles[i].Weight < 0.001 {
			pf.Particles[i].Weight = 0.001
		}
		totalWeigh += pf.Particles[i].Weight
	}
	if totalWeigh > 0 {
		for i := range pf.Particles {
			pf.Particles[i].Weight /= totalWeigh
		}
	}

}

// Systematic resampling https://people.isy.liu.se/rt/schon/Publications/HolSG2006.pdf
func (pf *ParticleFilter) Resample() {
	newParticles := make([]Particle, len(pf.Particles))

	cumWeights := make([]float64, len(pf.Particles))
	cumWeights[0] = pf.Particles[0].Weight

	for i := 1; i < len(pf.Particles); i++ {
		cumWeights[i] = cumWeights[i-1] + pf.Particles[i].Weight
	}

	step := 1.0 / float64(len(pf.Particles))
	start := pf.rng.Float64() * step

	idx := 0
	for i := 0; i < len(pf.Particles); i++ {
		target := start + float64(i)*step
		for idx < len(pf.Particles)-1 && cumWeights[idx] < target {
			idx++
		}

		newParticles[i] = pf.Particles[idx]
		newParticles[i].Lat += pf.gaussianNoise(0, 0.00001)
		newParticles[i].Lon += pf.gaussianNoise(0, 0.00001)
		newParticles[i].Heading += pf.gaussianNoise(0, 1.0)
		newParticles[i].Heading = osmprocessing.NormalizeBearing(newParticles[i].Heading)
		newParticles[i].Weight = 1.0 / float64(len(pf.Particles))
	}

	pf.Particles = newParticles
}

func (pf *ParticleFilter) MoveParticles(voReading VOReading) {
	for i := range pf.Particles {

		pf.Particles[i].Heading += voReading.Angle
		pf.Particles[i].Heading = osmprocessing.NormalizeBearing(pf.Particles[i].Heading)

		if voReading.Distance > 0 {
			newLat, newLon := osmprocessing.CalculateDestinationPoint(
				osmprocessing.MakeCoordinateDecimal(pf.Particles[i].Lat, osmprocessing.Latitude),
				osmprocessing.MakeCoordinateDecimal(pf.Particles[i].Lon, osmprocessing.Longitude),
				osmprocessing.BearingDecimal(pf.Particles[i].Heading),
				voReading.Distance,
			)

			pf.Particles[i].Lat = newLat.DecimalDegree
			pf.Particles[i].Lon = newLon.DecimalDegree
		}
	}
}
