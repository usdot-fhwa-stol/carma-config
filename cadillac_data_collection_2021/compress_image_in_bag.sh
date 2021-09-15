rosrun rosbag topic_renamer.py /hardware_interface/camera_fc/front_center/arena_camera_fc/image_raw tfhrc_loop_uncompressed.bag /hardware_interface/camera_fc/front_center/arena_camera_fc/uncomp/image_raw tfhrc_loop_uncompressed_renamed.bag
rosparam set /hardware_interface/camera_fc/front_center/arena_camera_fc/image_raw/compressed/format jpeg
rosparam set /hardware_interface/camera_fc/front_center/arena_camera_fc/image_raw/compressed/jpeg_quality 90
rosparam set /hardware_interface/camera_fc/front_center/arena_camera_fc/image_raw/compressed/png_level 9
rosbag play tfhrc_loop_uncompressed_renamed.bag

rosrun image_transport republish raw in:=/hardware_interface/camera_fc/frontenter/arena_camera_fc/uncomp/image_raw out:=/hardware_interface/camera_fc/front_center/arena_camera_fc/image_raw

rosbag record -a -O tfhrc_loop_jpeg_100.bag -x "(.*)/image_raw|(.*)/compressedDept(.*)|(.*)/theor(.*)"


rosbag play tfhrc_loop_jpeg_100.bag