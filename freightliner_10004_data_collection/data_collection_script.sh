cd /opt/carma/logs

rviz -d ~/carma_ws/carma/src/carma-config/freightliner_10004_data_collection/rviz/data_collection_view_cameras.rviz

rosbag record -e "/hardware_interface/camera(.*)/camera_info|/hardware_interface/camera(.*)/image_raw/compressed|/hardware_interface/radar_fc/as_tx/objects|/hardware_interface/radar_(.*)/as_tx/detections|/hardware_interface/radar_(.*)/as_tx/radar_markers|/hardware_interface/(.*)_report|/hardware_interface/gnss_fix_fused|/hardware_interface/lidar/points_raw|/joint_states|/rosout|/rosout_agg|/system_alert|/tf|/tf_static"
