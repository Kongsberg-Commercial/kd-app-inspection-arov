This directory contains the most up-to-date scripts used directly on the ROV.

Currently, there are two versions.

**multilateration.py** is the complete implementation of the positioning algorithm. This script includes CPAP communication, dynamic sound velocity calculation, sensor-data aquisition through pymavlink, position estimation with multilateration, and publishing of the point estimation to the GPS_INPUT message, allowing for the position to be displayed on the QGroundControl map.

**multilateration_with_depthcalc.py** is an updated version that implements a manual calculation of the depth. This is done because VFR_HUD.alt does not give us the depth reading after all. Instead, we use the pressure reading from SCALED_PRESSURE2 to calculate the depth. 
