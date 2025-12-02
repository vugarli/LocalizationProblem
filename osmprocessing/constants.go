package osmprocessing

import (
	"fmt"
	"math"
)

const R float64 = 6371e3

const (
	Latitude  = iota
	Longitude = iota
)

const (
	South = iota // -
	East  = iota // +
	North = iota // +
	West  = iota // -
)

// type Node osm.Node
// type Way osm.Way

type Coordinate struct {
	Deg       int
	Minute    int
	Second    int
	Direction int
}
type CoordinateDecimal struct {
	DecimalDegree float64
	CoordType     int
}

type BearingDecimal float64

func makeCoordinateDecimal(decimalDegree float64, coordType int) CoordinateDecimal {

	if coordType == Longitude {
		decimalDegree = normalizeLon(decimalDegree)
	}

	return CoordinateDecimal{DecimalDegree: decimalDegree, CoordType: coordType}
}

func (c CoordinateDecimal) String() string {
	coordinate := c.ToCoordinate()
	var direction rune

	switch coordinate.Direction {
	case South:
		direction = 'S'
	case North:
		direction = 'N'
	case West:
		direction = 'W'
	case East:
		direction = 'E'
	}

	return fmt.Sprintf("%d°%d'%d\" %c", coordinate.Deg, coordinate.Minute, coordinate.Second, direction)
}

func normalizeLon(lon float64) float64 {
	return math.Mod(lon+540, 360) - 180
}

func ToDecimalBearing(deg, min, sec int) BearingDecimal {
	return BearingDecimal(float64(deg) + float64(min)/60.0 + float64(sec)/3600.0)
}

func ToDecimalCoord(deg, min, sec, direction int) CoordinateDecimal {
	return Coordinate{Deg: deg, Minute: min, Second: sec, Direction: direction}.ToDecimalCoord()
}

func (c CoordinateDecimal) ToCoordinate() Coordinate {
	var direction int

	switch c.CoordType {
	case Latitude:
		if c.DecimalDegree >= 0 {
			direction = North
		} else {
			direction = South
		}
	case Longitude:
		if c.DecimalDegree >= 0 {
			direction = East
		} else {
			direction = West
		}
	}

	absDegree := math.Abs(float64(c.DecimalDegree))
	degree := int(absDegree)

	decMinute := (absDegree - float64(degree)) * 60
	minute := int(decMinute)

	decSeconds := (decMinute - float64(minute)) * 60
	seconds := int(math.Round(decSeconds))

	if seconds == 60 {
		minute++
		seconds = 0
	}
	if minute == 60 {
		degree++
		minute = 0
	}

	return Coordinate{Deg: degree, Minute: minute, Second: seconds, Direction: direction}
}

func (c Coordinate) ToDecimalCoord() CoordinateDecimal {

	decimalDegree := float64(c.Deg) + float64(c.Minute)/60.0 + float64(c.Second)/3600.0
	coordType := Latitude
	if c.Direction == West || c.Direction == South {
		decimalDegree *= -1
	}
	if c.Direction == East || c.Direction == West {
		decimalDegree = normalizeLon(decimalDegree)
		coordType = Longitude
	}

	return CoordinateDecimal{
		DecimalDegree: decimalDegree,
		CoordType:     coordType}
}
