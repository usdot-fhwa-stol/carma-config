# Basic Simulation Platform Vehicle

This configuration image contains the Compose definition and runtime files for
one ROS 2 CARMA Platform vehicle in a CDASim scenario.

It provides these services:

- `platform_ros2`
- `carma-carla-integration`
- `vehicle_registration_service`
- `v2x_ros_driver`

The configuration is stored at `/opt/carma/vehicle/config`. Scenario Runner
supplies the external simulation and vehicle-private network names, Docker
hostnames, runtime image versions, vehicle settings, and instance-specific V2X
Driver parameters.

The Compose file uses Docker DNS names instead of fixed container IP addresses.
Run `./build-image.sh` for a release build or `./build-image.sh --develop` for a
development build.
