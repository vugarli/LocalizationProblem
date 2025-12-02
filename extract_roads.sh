#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <input_file.pbf> <output_file.pbf>"
    exit 1
fi

in="$1"
out="$2"


osmium tags-filter "$in" nwr highway=motorway,trunk,primary,secondary,tertiary,
\ residential,service,unclassified,living_street,motorway_link,trunk_link,primary_link,
\ secondary_link,tertiary_link -o "$out" -O