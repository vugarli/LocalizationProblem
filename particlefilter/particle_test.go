package particlefilter

import (
	"roboticsproject/osmprocessing"
	"testing"
)

func TestBasicParticleInit(t *testing.T) {
	numberOfParticles := 4
	initLat := 1.0
	initLon := 1.0
	averageWeight := 1 / float64(numberOfParticles)

	particleFilter := NewParticleFilter(numberOfParticles, nil)
	particleFilter.InitParticles(initLat, initLon, 50)

	particles := particleFilter.Particles

	if len(particles) != numberOfParticles {
		t.Fatalf("Expected number of particles %v, but got %v", numberOfParticles, len(particles))
	}
	for i := 0; i < numberOfParticles; i++ {
		if particles[i].Weight != averageWeight {
			t.Fatalf("Expected average weight of %v, but got %v for particles(%v)", averageWeight, particles[i].Weight, i)
		}
	}
}

func TestMoveParticles(t *testing.T) {
	m, grid := osmprocessing.GenerateMap(3, 3, 200,
		osmprocessing.ToDecimalCoord(46, 0, 0, osmprocessing.North),
		osmprocessing.ToDecimalCoord(7, 0, 0, osmprocessing.East))

	em := osmprocessing.NewEnhancedMap(m)

	pf := NewParticleFilter(50, em)
	startNode := m.Nodes[grid["1,1"]]

	t.Run("particles move forward", func(t *testing.T) {
		pf.InitParticles(startNode.Lat, startNode.Lon, 1.0)

		initialPositions := make([]Particle, len(pf.Particles))
		copy(initialPositions, pf.Particles)

		voReading := VOReading{Distance: 10.0, Angle: 0}
		pf.MoveParticles(voReading)

		movedCount := 0
		for i, p := range pf.Particles {
			dist := osmprocessing.HaversineDistance(
				initialPositions[i].Lat, initialPositions[i].Lon,
				p.Lat, p.Lon)

			if dist > 5.0 && dist < 15.0 {
				movedCount++
			}
		}

		if movedCount < 45 {
			t.Errorf("only %d/50 particles correctly moved expected >45", movedCount)
		}
	})

	t.Run("particles move in their heading direction", func(t *testing.T) {
		for i := range pf.Particles {
			pf.Particles[i] = Particle{
				Lat:     startNode.Lat,
				Lon:     startNode.Lon,
				Heading: 0, // N
				Weight:  1.0 / 50,
			}
		}

		voReading := VOReading{Distance: 50.0, Angle: 0}
		pf.MoveParticles(voReading)

		movedNorth := 0
		for _, p := range pf.Particles {
			if p.Lat > startNode.Lat {
				movedNorth++
			}
		}

		if movedNorth < 45 {
			t.Errorf("Only %d/50 particles moved north expected >45", movedNorth)
		}
	})

	t.Run("zero distance does not move particles", func(t *testing.T) {
		pf.InitParticles(startNode.Lat, startNode.Lon, 1.0)

		initialPositions := make([]Particle, len(pf.Particles))
		copy(initialPositions, pf.Particles)

		voReading := VOReading{Distance: 0, Angle: 45}
		pf.MoveParticles(voReading)
		for i, p := range pf.Particles {
			dist := osmprocessing.HaversineDistance(
				initialPositions[i].Lat, initialPositions[i].Lon,
				p.Lat, p.Lon)

			if dist > 1.0 {
				t.Errorf("Particle %d moved %.2fm with zero distance", i, dist)
			}
		}
	})
}
func TestMotionUpdate_Turning(t *testing.T) {
	m, grid := osmprocessing.GenerateMap(3, 3, 200,
		osmprocessing.ToDecimalCoord(46, 0, 0, osmprocessing.North),
		osmprocessing.ToDecimalCoord(7, 0, 0, osmprocessing.East))

	em := osmprocessing.NewEnhancedMap(m)

	pf := NewParticleFilter(50, em)

	startNode := m.Nodes[grid["1,1"]]

	t.Run("particles turn right", func(t *testing.T) {
		for i := range pf.Particles {
			pf.Particles[i] = Particle{
				Lat:     startNode.Lat,
				Lon:     startNode.Lon,
				Heading: 0,
				Weight:  1.0 / 50,
			}
		}

		voReading := VOReading{Distance: 0, Angle: 90}
		pf.MoveParticles(voReading)

		headingEast := 0
		for _, p := range pf.Particles {
			if p.Heading > 70 && p.Heading < 110 {
				headingEast++
			}
		}

		if headingEast < 40 {
			t.Errorf("only %d/50 particles heading east expected > 40", headingEast)
		}
	})

	t.Run("particles turn left", func(t *testing.T) {
		for i := range pf.Particles {
			pf.Particles[i] = Particle{
				Lat:     startNode.Lat,
				Lon:     startNode.Lon,
				Heading: 0,
				Weight:  1.0 / 50,
			}
		}

		voReading := VOReading{Distance: 0, Angle: -90}
		pf.MoveParticles(voReading)

		headingWest := 0
		for _, p := range pf.Particles {
			if p.Heading > 250 && p.Heading < 290 {
				headingWest++
			}
		}

		if headingWest < 40 {
			t.Errorf("only %d/50 particles heading west expected > 40", headingWest)
		}
	})

	t.Run("heading wraps around 360", func(t *testing.T) {
		for i := range pf.Particles {
			pf.Particles[i] = Particle{
				Lat:     startNode.Lat,
				Lon:     startNode.Lon,
				Heading: 350,
				Weight:  1.0 / 50,
			}
		}

		voReading := VOReading{Distance: 0, Angle: 20}
		pf.MoveParticles(voReading)

		validHeadings := 0
		for _, p := range pf.Particles {
			if p.Heading >= 0 && p.Heading < 360 {
				validHeadings++
			}

			if p.Heading < 30 || p.Heading > 350 {
				validHeadings++
			}
		}

		if validHeadings < 50 {
			t.Error("heading did not wrap correctly around 360!!")
		}
	})

	t.Run("heading wraps around 0", func(t *testing.T) {
		for i := range pf.Particles {
			pf.Particles[i] = Particle{
				Lat:     startNode.Lat,
				Lon:     startNode.Lon,
				Heading: 10,
				Weight:  1.0 / 50,
			}
		}

		voReading := VOReading{Distance: 0, Angle: -20}
		pf.MoveParticles(voReading)

		validHeadings := 0
		for _, p := range pf.Particles {
			if p.Heading >= 0 && p.Heading < 360 {
				validHeadings++
			}

			if p.Heading > 330 || p.Heading < 10 {
				validHeadings++
			}
		}

		if validHeadings < 50 {
			t.Error("heading didn't wrap correctly!!")
		}
	})
}
