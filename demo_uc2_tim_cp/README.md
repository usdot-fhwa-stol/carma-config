# TFHRC Demo Configuration — UC2 / Traffic Incident Management / Cooperative Perception

This CARMA Platform configuration supports a multi-scenario demo at the Turner–Fairbank Highway Research Center (TFHRC). The demo includes three use cases across the TFHRC campus:

- **MAP & SPaT Adherence** at the East intersection
- **Traffic Incident Move-Over** on Innovation Drive
- **Cooperative Perception** at the West intersection

## Vehicle

This configuration is built for the **2019 Chrysler Pacifica**. It can be adapted to other vehicles, but additional calibration and testing would be required.

## Docker Images

The platform service uses a custom CARMA Platform image:

```
usdotfhwastoldev/carma-platform:demo_uc2_tm_cp
```

All other services use the standard `carma-system-4.11.0` release images, as defined in the `.env` file:

```
DOCKER_ORG=usdotfhwastol
DOCKER_TAG=carma-system-4.11.0
```

## Maps and Routes

The route and map files for this demo are located on the vehicle at:

```
/opt/carma/vehicle/demo_uc2_tm_cp
```

The `docker-compose.yml` mounts this directory as both `/opt/carma/maps` and `/opt/carma/routes` inside the platform container.


## Notable Parameter Overrides

The `GlobalParamsOverride.yaml` sets several non-default values required for this demo:

- Localization mode set to `5` with position offsets (`x: 2.2`, `y: -2.1`)
- Plan delegator `max_traj_generation_reattempt` increased to `1000`
- LCI strategic plugin buffers adjusted for intersection approach behavior
- `trajectory_time_length`in InlaneCruising is set to 12.0 to allow for long trajectories.

## Installation

The config image can either be built locally or pulled from DockerHub.

Once the config image is available, follow the standard CARMA config workflow:

1. Download all required images:
```bash
   carma config install <image_name>
```

2. Set the active configuration:
```bash
   carma config set <image_name>
```

3. Start the platform:
```bash
   carma start all
```
