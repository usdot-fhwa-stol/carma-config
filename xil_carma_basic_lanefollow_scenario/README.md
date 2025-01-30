# CDASim CARMA Basic Lanefollow Scenario

## Introduction

This **CARMA Config** includes the docker-compose and configuration file setup for the **CDASim with CARMA Basic Lanefollow Scenario**.

## Scenario Description

This CARMA Configuration Image creates a **XIL** (Anything-In-the-Loop) scenario which includes **CARLA**, **SUMO**, and **CARMA Platform**. The scenario configured is meant to show base basic CDASim functionality. There are other configurations that enable infrastructure computation and communication, traffic control, and carma-cloud, which are not included here. That said, these components can be added to this configuration following the patterns from other configurations. This component provides an interface for CDA participants to interact with the road infrastructure.

![Alt text](docs/Town04_map.png)

![Alt text](docs/vehicle_1_route.png)

![Alt text](docs/vehicle_2_route.png)

## Simulators

| Simulator      | Version |
| ----------- | ----------- |
| CARLA      | 0.9.10       |
| SUMO      | 1.15       |

## Deployment Instructions

1) Copy all files in the `cdasim_config/route_config` directory to directory to `/opt/carma/routes/`
2) Copy the osm map in `cdasim_config/carma/` directory to `/opt/carma/maps/` and create a symbolic link to it named `vector_map.osm`
3) Install carma-script extension (see instructions in [Setup CARMA Scripts](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/488472599/Setup+CARMA+Platform+Runtime))
4) Build or pull carma-config image and run `carma config set <image_name>`
