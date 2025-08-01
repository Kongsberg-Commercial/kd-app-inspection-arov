## Deployment Guide
This repository contains scripts and instructions to deploy the end-to-end data flow between the MQTT broker, HiPOS and the ROV in order to position the ROV regarding the USV (RAN).    

##  Required scritps  
Launch files:  
launch_all.bat  
lunc_topside

  
## Software Requirements  
-HIPOS software downloaded with correctly setup on the computer onboard the boat.  
-Active license for the adapter between the ethernet cabel and the computer (tailscle IP adress: 100.65.237.59 with name kpc22033722-rov-computer).  
-QGroundControl Software downloaded and set up correctly.   

## Setup & Run  
-Installing and running HIPOS Software on the windows computer (tailscle IP adress: 100.65.237.59 with name kpc22033722-rov-computer).  
-Set up a conda environment named 

## Scripts  
-Run the launch file on the kpc22033722-rov-computer called "launch_all.bat"  
Open a terminal on the computer find the path where the launch file is located by using the following command:  
<pre> cd \kd-app-inspection-arov\lauch_all.bat </pre>

Then you should launch the launch file by using the following command:  
<pre> launch_all.bat </pre>  
In the script change set REMOTE_IP=100.127.125.86 to the tailscale ip of the remote computer
also in the launch file change some of the python environments under the rov2 conda enviornment to your python path. this you can find running the where python command
Ask chatgpt how to install GSTREAMER, and opencv with ffplay support to run inside the conda environment

Remeber to input password over ssh for the pi

## Qgroundcontrol setup
Under comm links add link, change Server Adresses to your tailscale ip and change the port to 14551
Under video change udp url to your_tail_scail_ip:5600

