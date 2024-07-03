# DTRG - PX4 Autopilot Firmware
<a href="https://dtrg.org" style="padding:20px" ><img src="https://dtrg.org/wp-content/uploads/2023/03/drone_logo-v2-300x81.png" alt="DTRG Logo" width="250px"/></a>

This repository holds the [DTRG](https://dtrg.org/) fork of the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

This firmware is build upon the v1.14 release of PX4
* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))

## DTRG Additions

The DTRG fork of PX4 includes the following additions:

Implemented:
* Mixer Injection from csv file
* Manual Horizontal Thrust Control via RC

Upcoming:
* Dynamixel Servo Control with easy configuration
* Wrench Estimator
* Admittance Controller

This DTRG fork is designed with the following airframes in mind:

* BlueOcto (stacked octocopter)
* PlanarOcto (flat octocopter)
* Planatary Hex (modified flat hexacopter)

Documentation for features implemented in v1.14 can be found in the DTRG teams drive documentation folder. It is highly recommended to read the documentation before attempting to use the firmware as some features may pose a safety risk if not used correctly.

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>
<div style="padding:10px">&nbsp;</div>
