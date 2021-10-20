cd /media/carma/Storage

rviz -d /home/carma/carma_ws/carma/src/carma-config/cadillac_data_collection_2021/rviz/data_collection_view_cameras.rviz

rosbag record -b 16384 -e "|/can/(.*)|/hardware_interface/camera(.*)/camera_info|/hardware_interface/camera(.*)/image_raw/compressed|/hardware_interface/radar_(.*)/delphi_esr/sensor/objects|/hardware_interface/lidar/points_raw|/hardware_interface/comms/(.*)|/hardware_interface/gnss_fix_fused|/joint_states|/rosout|/rosout_agg|/system_alert|/tf|/tf_static"
