package main

import (
	"fmt"
	"roboticsproject/osmprocessing"
)

func main() {

	objects := osmprocessing.ExtractObjects("X:/Documents/RoboticsProjectGo/Bordeaux.osm.pbf", true)
	objects.SaveObjects("filtered")
	fmt.Println(len(objects.Nodes))
	obj2 := &osmprocessing.Map{}
	obj2.LoadObjects("filtered.json")
	fmt.Println(len(obj2.Nodes))
	fmt.Println(len(obj2.Ways))
	osmprocessing.SaveMapToOSM(obj2, "filtered")
}
