#!/bin/bash

#This script  utilizes sumo od2trips ( https://sumo.dlr.de/docs/od2trips.html ) tool to generate background traffic for Town05 given Traffic assignment zones (TAZ) and OD matrix

#To change the number of vehicles in the horizontal or vertical direction, modify the values in the third column of OD_file.od, which specify the number of vehicles, and then re-run the generate_route.sh script to generate the route file with the updated vehicle count. The first two columns in the OD matrix specify the origin and destination, while the third column defines the number of vehicles traveling between them. For a detailed guideline, refer to SUMO od2trips documentation here  https://sumo.dlr.de/docs/od2trips.html

#This script requires Sumo tools od2trips and duarouter and sumo tools can installed  using the command:  sudo apt-get install sumo-tools (https://sumo.dlr.de/docs/Installing/index.html)

od2trips -c od2trips.config.xml -n Taz.xml --od-matrix-files OD_file.od -o od_file.odtrips.xml

duarouter -c dua_router.cfg

#removing extra route files

rm od_file.odtrips.xml Town05.rou.alt.xml

# Carla Passenger vehicle classes to add carla vehicle types in Sumo routes

vehicle_classes=(
  "vehicle.audi.a2"
  "vehicle.audi.tt"
  "vehicle.jeep.wrangler_rubicon"
  "vehicle.chevrolet.impala"
  "vehicle.mini.cooperst"
  "vehicle.mercedes-benz.coupe"
  "vehicle.bmw.grandtourer"
  "vehicle.citroen.c3"
  "vehicle.mustang.mustang"
  "vehicle.lincoln.mkz2017"
  "vehicle.seat.leon"
  "vehicle.nissan.patrol"
  "vehicle.nissan.micra"
)

input_file="Town05.rou.xml"  
temp_file="temp.xml"

# Adding carla vehicle types in routes
while IFS= read -r line; do
  if [[ "$line" == *'<vehicle '* ]]; then
    # Select a random vehicle type from carla vehicle class
    random_vehicle=${vehicle_classes[$RANDOM % ${#vehicle_classes[@]}]}
    
    # Add type attribute before the closing '>'
    line=$(echo "$line" | sed -E "s/(<vehicle [^>]+)/\1 type=\"$random_vehicle\"/")
  fi
  echo "$line" >> "$temp_file"
done < "$input_file"

# Replace the original file with the updated file
mv "$temp_file" "$input_file"

echo "Background route file Town05.rou.xml has been successfully generated"


