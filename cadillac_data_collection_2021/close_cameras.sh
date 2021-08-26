
## The lucid cameras with their arena camera ros driver work fine when run locally, but have some issues with docker. 

# Issue 1: Only one works at a time in a single container. Probably a networking/ports issue, but for now each cameras gets its own docker container. 
# Issue 2: killing carma start all (^C or carma stop all) doesn't properly stop the cameras. To solve this, this script goes into each camera container and properly shuts down the camera. 


echo "Safely closing cameras..."
docker exec -it arena-camera-driver-fl bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_fl/front_left/arena_camera_fl"
docker exec -it arena-camera-driver-rl bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_rl/rear_left/arena_camera_rl"
docker exec -it arena-camera-driver-rc bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_rc/rear_center/arena_camera_rc"
docker exec -it arena-camera-driver-rr bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_rr/rear_right/arena_camera_rr"
docker exec -it arena-camera-driver-fr bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_fr/front_right/arena_camera_fr"
docker exec -it arena-camera-driver-fc bash -c "source /opt/ros/kinetic/setup.bash && rosnode kill /hardware_interface/camera_fc/front_center/arena_camera_fc"

