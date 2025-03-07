This directory contains the most up-to-date scripts used directly on the ROV.

Currently, there are two versions.

**multilateration.py** is the complete implementation of the positioning algorithm. This script includes CPAP communication, dynamic sound velocity calculation, sensor-data aquisition through pymavlink, position estimation with multilateration, and publishing of the point estimation to the GPS_INPUT message, allowing for the position to be displayed on the QGroundControl map.

**multilateration_with_fallback.py** is an updated version that implements a fallback feature, handling the case where one transponder interrogation times out. We can still perform multilateration with slightly reduced accuracy by copying one of the other valid distances and corresponding transponder-positions
