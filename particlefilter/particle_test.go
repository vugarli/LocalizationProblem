package particlefilter

import (
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

// func TestParticleInitRoads(t *testing.T) {
// 	numberParticlesPerRoad := 5

// }
