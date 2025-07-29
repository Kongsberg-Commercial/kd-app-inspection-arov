## Deployment Guide
This repository contains scripts and instructions to deploy the end-to-end data flow between the MQTT broker, HiPOS and the ROV in order to position the ROV regarding the USV (RAN).    

##  Required scritps  
# Launch files:  
launch_all.bat  
lunc_topside

  
## Software Requirements  
-HIPOS software downloaded with correctly setup on the computer onboard the boat.  
-Active license for the adapter between the ethernet cabel and the computer (tailscle IP adress: 100.65.237.59 with name kpc22033722-rov-computer).  
-QGroundControl Software downloaded and set up correctly.   

## Setup & Run  
-Installing and running HIPOS Software on the windows computer (tailscle IP adress: 100.65.237.59 with name kpc22033722-rov-computer).  
-Set up  

## Scripts  
-Run the launch file on the kpc22033722-rov-computer called "launch_all.bat"  
Open a terminal on the computer find the path where the launch file is located by using the following command:  
<pre> cd Documents/microPAP </pre>

Then you should launch the launch file by using the following command:  
<pre> launch_all.bat </pre>  
This will launch the two scrips hipos_interface.py and mqtt_pposition.py.  
