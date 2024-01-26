| DockerHub Release | DockerHub Release Candidate | DockerHub Develop |
|------|-----|-----|
[![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastol/carma-config?label=Docker%20Build&logo=232496ED)](https://hub.docker.com/repository/docker/usdotfhwastol/carma-config) | [![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastolcandidate/carma-config?label=Docker%20Build&logo=232496ED)](https://hub.docker.com/repository/docker/usdotfhwastolcandidate/carma-config) | [![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastoldev/carma-config?label=%20Carma-config)](https://hub.docker.com/repository/docker/usdotfhwastoldev/carma-config)

# CARMAConfig
The CARMAConfig repository stores the vehicle-specifc and vehicle-class specific configuration files (such as Docker Compose manifests, network configuration files, system parameters) for use with the different vehicles the CARMA Platform supports. A detailed breakdown of how CARMA treats vehicle configuration can be found on the confluence page: https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/196182019/CARMA3+Project+Documentation?preview=/196182019/198574135/CARMA%20Platform%20Detailed%20Design%20-%20Parameter%20and%20Launch%20Standards%20for%20Different%20Vehicle%20Configurations.docx

## Vehicle Configuration Folders
Folders containing the name of a specific vehicle class such as lexus_rx_450h_2019 contain vehicle configuration data that is specific to that class of vehicle and sensor suite. For example the launch file for the sensors used on that vehicle. These folders also contain the docker compose files that will launch CARMA with the appropriate configuration. These folders do not contain calibration information that is specific to the individual vehicles. 

## Vehicle Calibration Folder
Some parameters are unique to individual vehicles such as precise sensor orientations or controller tunings. These values are stored separately from parameters that apply to classes of vehicles. A special folder structure is used for this purpose, an example of this structure can be seen in the example_calibration_folder directory. Calibration folders should be installed on the vehicle such that their vehicle folders can be found at /opt/carma/vehicle. It is recommended that this be done as a sym-link to a git repository so your vehicle calibration data can be version controlled.

## Example Opt Folder

The CARMA Platform requires that some files be located in the /opt/carma directory so they can be found at runtime. The example_opt_carma folder in this repository contains an example of this folder's structure though some files cannot be included in this repo due to size or license restrictions. Therefore, the installation instructions should be consulted for proper setup. The folder is presented here as a supporting reference and used by development setup scripts. 


## Carla Recorder

This **CARMA Config** includes volumes and images that will use **Carla's** [recorder](https://carla.readthedocs.io/en/0.9.10/adv_recorder/) functionality to record **CARLA** simulation data into a `carla-recorder/` directory. Included will be a `Trb2024_1.json` file and a `Trb2024_1.log` file. The name of the file comes from the scenario name define in the **scenario-runner** image. The `.json` file is a criteria file and the `.log` file the carla simulation recording (see https://carla-scenariorunner.readthedocs.io/en/latest/metrics_module/). Using the metrics module in carla-scenario-runner we can define metrics in carla and evaluate the metrics from the carla recordings. This works for collision monitoring and will be prototyped for a custom defined near miss metric.