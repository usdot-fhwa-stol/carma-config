# ROS Version Status of CARMA Platform and CARMA Messenger Components

## Overview

Currently CARMA Platform's ROS2 (foxy) migration from ROS1 (noetic) is almost complete (and upgrade from foxy to humble is underway).
    However, as it can inspected from the configurations in this repository, some machines still require ROS1 components to run (hence ros1_bridge to bridge between the two versions).

Currently CARMA Platform and CARMA Messenger are running ROS1 and ROS2 hybrid approach. Therefore, typical docker image structure for each machine looks as follows:
```
docker-compose.yml:

- carma-base image running roscore
- carma-platform image running ros1 non-driver components
- carma-platform image running ros2 non-driver components
- carma-msgs image running ros1_bridge
- ... other ros1 or ros2 drivers and services
```

This document outlines the reasoning behind such requirements. Following table shows a quick overview of what machines are capable of running which ROS version as their drivers and components. By utilizing docker containerization, each components can run different versions of ROS:

![](image/ROS%20table.png)

## Non-Driver CARMA Platform Components

These include all the non-driver nodes that runs in carma-platform. All launch files specifying the nodes can be found [here](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/carma/launch). These, for example, include all localization, planning, or communication nodes that connect with driver or driver wrapper nodes. By utilizing ros1_bridge to bridge the topics and services from some remaining ROS1 components, carma-platform is able to accommodate certain driver limitations while still running majority in ROS2.

**NOTE:** Despite showing ROS2 uniformly as its version, only one non-critical package called [carma_record](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/carma_record), that creates `.rosbag` files, is in ROS1 at the moment to support data analysis while the team migrates data analysis scripts to ROS2. As a side note, since majority of the topics and services are in ROS2 and don't require to be bridged to ROS1, these need to be explicitly bridged such [as here](https://github.com/usdot-fhwa-stol/carma-config/blob/develop/development/bridge.yml) to be saved in a `.rosbag`.

## Local Development

[Local development](https://github.com/usdot-fhwa-stol/carma-config/tree/develop/development) simulates vehicle sensors as mock drivers. These mock_drivers publish its respective status (simulated) to `/hardware_interdace/driver_discovery` just like real drivers, which is how CARMA Platform monitors the health of its drivers. [This package](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/mock_drivers) is currently in ROS1, which is why all drivers indicate ROS1. it is expected to be converted soon.

## Camera Driver

As it shows in the table, all drivers are in ros1. However, currently camera sensor integration is not thoroughly tested; therefore all machines are running mock camera drivers, which is in ROS1 as previously mentioned.

## Speed and Steering Controller (SSC) Node and Driver

Previously, ROS1 SSC nodes that interface with the SSC driver were provided by Hexagon. These nodes reduce the complexity interfacing with the controllers and smooth the steering, throttle, and brake commands sent to the controller. However, as of August 2023, Hexagon has no plans to migrate these nodes to ROS2, except that of Lexus RX 450h, which is in ROS2. Therefore, currently:

- Lexus uses pacmod controller, ROS2 SSC node, and ROS2 carma ssc interface wrapper
- Chrysler Pacifica uses New Eagle controller, ROS1 SSC node, and ROS1 carma ssc interface wrapper
- Ford Fusion uses Dataspeed controller, ROS1 SSC node, and ROS1 carma ssc interface wrapper
- Freightliner uses pacmode controller, ROS1 SSC node, and ROS1 carma ssc interface wrapper

Docker containerization and ros1_bridge allows CARMA Platform to run different versions of ssc drivers successfully. Since this hybrid approach can accommodate normal operation of CARMA Platform, CARMA Platform has no plans to migrate these to ROS2 as well at the moment.

## XiL drivers

As it shows in the table, all "drivers" are in ROS1. This is due to [CDASim](https://github.com/usdot-fhwa-stol/cdasim)'s limitation at the moment to use CARLA 0.9.10. This CARLA version only exposes ROS1 bridge. Therefore, all sensors are simulated in CARLA, which exposes ROS1 data that can be bridged to ROS2. Only communication data such as DSRC or C-V2X is simulated using ns3. Its [corresponding adapter](https://github.com/usdot-fhwa-stol/carma-ns3-adapter) is in ROS1 at the moment and will be converted to ROS2 in the future ( most likely around the same time CARLA is upgraded).

## Pinpoint Driver

Currently, the driver has been ported to ROS2; however, it has not been tested yet.

## Lidar Lite Angle Driver

TODO
