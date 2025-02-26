od2trips -c od2trips.config.xml -n Taz.xml --od-matrix-files OD_file.od -o od_file.odtrips.xml

duarouter -c dua_router.cfg
rm od_file.odtrips.xml Town05.rou.alt.xml



# List of possible vehicle types
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

input_file="Town05.rou.xml"  # Change this to your actual file name
temp_file="temp.xml"

# Process the file line by line
while IFS= read -r line; do
  if [[ "$line" == *'<vehicle '* ]]; then
    # Select a random vehicle type
    random_vehicle=${vehicle_classes[$RANDOM % ${#vehicle_classes[@]}]}
    
    # Add type attribute before the closing '>'
    line=$(echo "$line" | sed -E "s/(<vehicle [^>]+)/\1 type=\"$random_vehicle\"/")
  fi
  echo "$line" >> "$temp_file"
done < "$input_file"

# Replace the original file with the updated file
mv "$temp_file" "$input_file"

echo "Background route file Town05.rou.xml has been sucssfully generated"


