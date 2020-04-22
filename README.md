# CARMAConfig
The CARMAConfig repository stores the vehicle-specifc and vehicle-class specific configuration files (such as Docker Compose manifests, network configuration files, system parameters) for use with the different vehicles the CARMA Platform supports. A detailed breakdown of how CARMA treats vehicle configuration can be found on the confluence page: https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/196182019/CARMA3+Project+Documentation?preview=/196182019/198574135/CARMA%20Platform%20Detailed%20Design%20-%20Parameter%20and%20Launch%20Standards%20for%20Different%20Vehicle%20Configurations.docx

## Vehicle Configuration Folders
Folders containing the name of a specific vehicle class such as lexus_rx_450h_2019 contain vehicle configuration data that is specific to that class of vehicle and sensor suite. For example the launch file for the sensors used on that vehicle. These folders also contain the docker compose files that will launch CARMA with the appropriate configuration. These folders do not contain calibration information that is specific to the individual vehicles. 

## Vehicle Calibration Folder
Some parameters are unique to individual vehicles such as precise sensor orientations or controller tunings. These values are stored separately from parameters that apply to classes of vehicles. A special folder structure is used for this purpose, an example of this structure can be seen in the example_calibration_folder directory. Calibration folders should be installed on the vehicle such that their vehicle folders can be found at /opt/carma/vehicle. It is recommended that this be done as a sym-link to a git repository so your vehicle calibration data can be version controlled.

## Example Opt Folder

The CARMA Platform requires that some files be located in the /opt/carma directory so they can be found at runtime. The example_opt_carma folder in this repository contains an example of this folders structure though some files cannot be included in this repo due to size or license restrictions. Therefore the installation instructions should be consulted for proper setup. The file is presented here as a supporting reference and used by development setup scripts. 

# CARMAPlatform
The primary CARMAPlatform repository can be found [here](https://github.com/usdot-fhwa-stol/CARMAPlatform) and is part of the [USDOT FHWA STOL](https://github.com/usdot-fhwa-stol/)
github organization. Documentation on how the CARMAPlatform functions, how it will evolve over time, and how you can contribute can be found at the above links as well

## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Contributing.md) 

## Code of Conduct 
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/ATTRIBUTION.txt) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/docs/License.md) 

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/CARMAPlatform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
