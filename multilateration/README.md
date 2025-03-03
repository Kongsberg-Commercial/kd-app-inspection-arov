#### Multilateration

This project contains a series of different multilateration tests. 
We have the raw mathematical simulation for the algorithm in multilateration_algorithm.py. 
This script uses real transponder coordinates in lat/lon and simulated ROV movement, along with multilateration to show the proof-of-concept for the algorithm

There are four subdirectories in this folder:
- multilateration_no_depth
- multilateration_static_depth
- multilateration_with_mavlink
- serialport_simulation

There is a lot of shared code between these versions, so if one problem arises in one version during testing, the same problem may be present in other versions. 

Firstly, the **serialport_simulation** aims to simulate the serial communication with the cPAP for when physical testing isn't possible. This can be useful if logic needs testing without access to the ROV.
There are two scripts in this directory, one to simulate the onboard ROV logic for communication and position estimation, while the other script is responsible for simulating the ROV's (or rather the cPAP's) movement, while also handling cPAP - cNODE communication 
More details can be found in the subdirectory's own README.md file.


**multilateration_no_depth** is a version of the algorithm that does not use the depth-readings (z) to determine the position. This reduces the accuracy of the algorithm and requires that one of the transponders is positioned further down in the water to solve a geometric discrepency (or else it would be a 2D problem)
This script *should* be able to be run directly on the ROV to get an estimate of the ROV's position. More details can be found in the subdirectory's own README.md file.


**multilateration_static_depth** is a version of the algorithm that uses a fixed depth-readings (z) to determine the position. This solves the geometric discrepency with no depth readings, but gets more inaccurate if the ROV is not positioned at the hard-coded depth.
This script *should* be able to be run directly on the ROV to get an estimate of the ROV's position. More details can be found in the subdirectory's own README.md file.


**multilateration_with_mavlink** is a version of the algorithm that uses live depth-readings (z) from mavlink to determine the position. This should be the most accurate and useful version for testing the transponders, however, the mavlink reading have not *actually* been tested live. This is a bit of a hail-mary shot.
This script *should* be able to be run directly on the ROV to get an estimate of the ROV's position. More details can be found in the subdirectory's own README.md file.

