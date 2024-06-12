# K900 CARMA Cloud CDASim Integration Scenario

## Introduction

This **CARMA Config** includes the docker-compose and configuration file setup for the **K900 CARMA Cloud CDASim Integration Scenario**.

## Scenario Description

This CARMA Configuration Image creates a **XIL** (Anything-In-the-Loop) scenario which includes **CARLA**, **SUMO** , **NS3** (CV2X Model), a **Virtual Signal Controller**, **CARMA Streets** and **CARMA Platform**. The scenario configured is meant to show base basic CDASim functionality along with the new developed **CARMA Cloud** integration. **CARMA Cloud** is a cloud application used to maintain and provide traffic controls for autonomous vehicles to use to update their local maps. The diagrams below illustrate the configured scenario including vehicle routes and infrastructure location.


![Alt text](docs/town04_diagram.png)


![Alt text](docs/scenario_diagram.png)

## Simulators

| Simulator      | Version |
| ----------- | ----------- |
| CARLA      | 0.9.10       |
| SUMO      | 1.15       |

## Deployment Instructions

1) Copy all files in the `cdasim_config/route_config` directory to  directory to `/opt/carma/routes/`
2) Copy the osm map in `cdasim_config/carma/` directory to `/opt/carma/maps/` and create a symbolic link to it named `vector_map.osm`
3) Unzip [carmacloud.zip](https://leidoscorpus.sharepoint.us/sites/STR/TODevelopmentLibrary/Forms/AllItems.aspx?OR=Teams%2DHL&CT=1718132983172&clickparams=eyJBcHBOYW1lIjoiVGVhbXMtRGVza3RvcCIsIkFwcFZlcnNpb24iOiI0OS8yNDA1MDMwNzYxNCIsIkhhc0ZlZGVyYXRlZFVzZXIiOmZhbHNlfQ%3D%3D&id=%2Fsites%2FSTR%2FTODevelopmentLibrary%2F22%2D241%20CARMA%20XIL%202%2FWork%2DIn%2DProgress%2FTask%202%20CARMA%20Cloud%20Integration%2Fcarmacloud%2Ezip&viewid=203b55b8%2D6a07%2D4b3b%2D8097%2D4a10dba36afb&parent=%2Fsites%2FSTR%2FTODevelopmentLibrary%2F22%2D241%20CARMA%20XIL%202%2FWork%2DIn%2DProgress%2FTask%202%20CARMA%20Cloud%20Integration) in `cdasim_config/` to create `cdasim_config/carmacloudvol/` with traffic controls and xodr map.

> [!NOTE]  
> These **CARMA Cloud** traffic controls and xodr map only for CARLA Town04. New ones must be generated for other CARLA Maps.

4) Build Virtual Signal Controller image locally (see instructions in [Setup CARMA Config](https://usdot-carma.atlassian.net/wiki/spaces/CRMSIM/pages/2526937089/Setup+Instructions+Documentation+CARMA-Streets+Integration) step 1 )
5) Install carma-script extension (see instructions in [Setup CARMA Scripts](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/488472599/Setup+CARMA+Platform+Runtime))
6) Build or pull carma-config image and run `carma config set <image_name>`

> [!NOTE]  
> Suggestion: to build both 1 vehicle and 2 vehicle carma-config images by commenting out the `carma_2` in the docker compose. `docker build -t carma-config:<one_vehicle or two_vehicle> .` in this directory.

7) Navigate to the `cdasim_config/` directory and run the `start_simulation` script
8) To stop the simulation simply run the `stop_simulation` script in the same directory (ensure any kafka data from carma streets is collected before stopping simulation)
9) This scenario can generate a lot of logged data, using the `clear_logs` script in the same directory will clear **CDASim**, **CARMA Streets**, and **CARMA Platform Logs**

