# K900 CARMA Cloud CDASim Integration scenario Scenario

## Introduction

This **CARMA Config** includes the docker-compose and configuration file setup for the **K900 CARMA Cloud CDASim Integration scenario Scenario** scenario.

## Scenario Description

This CARMA Configuration Image creates a **XIL** (Anything-In-the-Loop) scenario which includes **CARLA**, **SUMO** , **NS3** (CV2X Model), a **Virtual Signal Controller**, **CARMA Streets** and **CARMA Platform**. The scenario configured is meant to show base basic CDASim functionality along with the new developed **CARMA Cloud** integration. **CARMA Cloud** is a cloud application used to maintain and provide traffic controls for autonomous vehicles to use to update their local maps. The diagrams below illustrate the configured scenario including vehicle routes and infrastructure location.

![Alt text](docs/carma_1_route.png)![Alt text](docs/carma_2_route.png)
TODO Add diagram of RSU location (VRU location currently)
## Simulators

| Simulator      | Version |
| ----------- | ----------- |
| CARLA      | 0.9.10       |
| SUMO      | 1.15       |

## Deployment Instructions

1) Copy all files in the `cdasim_config/routes` directory to  directory to `/opt/carma/routes/`
2) Copy the osm map in `cdasim_config/carma/` directory to `/opt/carma/maps/` and create a symbolic link to it named `vector_map.osm`
3) Build Virtual Signal Controller image locally 
4) Install carma-script extension
5) Build or pull carma-config image and run `carma config set <image_name>`
6) Navigate to the `cdasim_config/` directory and run the `start_simulation` script
7) To stop the simulation simply run the `stop_simulation` script in the same directory (ensure any kafka data from carma streets is collected before stopping simulation)
8) This scenario can generate a lot of logged data, using the `clear_logs` script in the same directory will clear **CDASim**, **CARMA Streets**, and **CARMA Platform Logs** 

