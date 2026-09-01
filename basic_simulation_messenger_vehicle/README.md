# Basic Simulation Messenger Vehicle

This configuration image contains the Compose definition and runtime files for
one ROS 2 CARMA Messenger vehicle in a CDASim scenario.

It provides these services:

- `messenger-ros2`
- `carma-messenger-bridge`
- `messenger-v2x-ros-driver`
- `messenger-vehicle-registration-service`
- `carma-messenger-vehicle-plugin`

The configuration is stored at `/opt/carma/vehicle/config`, which is the path
used by the CARMA Messenger runtime images for launch files and parameters.

Scenario Runner extracts the complete configuration tree beside
`docker-compose.yml`. Relative bind mounts such as
`./carma_messenger/traffic_incident/config` intentionally map those extracted
files into the runtime image's `/opt/carma/install/...` location. The bundled
V2X driver parameters are a single-vehicle fallback; Scenario Runner replaces
that mount with an instance-specific parameter file at runtime.

Run `./build-image.sh` for a release build or `./build-image.sh --develop` for a
development build.
