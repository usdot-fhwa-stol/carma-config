# Basic Simulation Vehicle CARMA Config
This is a configuration image for basic carma vehicles in simulation. It only includes `/opt/carma/vehicle/config/` files and not CARMA Platform plugin configuration files since CARMA Config does not currently handle plugin configuration files. The `docker-compose.yml` defined in this directory includes bridge images for CARMA Platform integration into CDASim including
- NS3 Adapter
- Carma Carla Integration