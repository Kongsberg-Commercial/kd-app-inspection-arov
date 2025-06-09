# Digital Twin for BlueROV2 – Bachelor Project  
This Unity-based digital twin was developed as part of our bachelor project at the University of South-Eastern Norway, in collaboration with Kongsberg Discovery. 
The digital twin visualizes real-time telemetry and positioning data from an ROV (Remotely Operated Vehicle) system and supports testing and monitoring of underwater sensor technologies.

## Features  
-Real-time ROV position and orientation visualization  
-MQTT integration for telemetry data streaming  
-Simulated subsea environment with terrain and marine assets  
-Digital representation of BlueROV2  
-Pipe-following visualization and acoustic positioning feedback  
-Built-in Unity using C# scripting and MQTT libraries  

## Requirements  
-Unity 2021.3 LTS or later (tested on Unity 2021.3.X)  
-MQTT Broker   
-ROV system running MQTT-compatible telemetry (e.g., Raspberry Pi with MQTT publisher)  
-.NET Standard 2.0-compatible MQTT library (e.g., MQTTnet)  

## Project Structure  
/Assets  
  /Scripts            -> C# scripts for data parsing and object control  
  /Prefabs            -> BlueROV2 model and components  
  /Scenes             -> Main Unity scenes  
  /Materials          -> Visual assets and effects  
README.md  

Getting Started  
Clone the repository   
Open the project in Unity Hub.  
Make sure your MQTT broker is running and configured to accept telemetry data.  
In Unity, press Play to start the simulation.   

MQTT Topics Used  
/pipe – Pipe-following controller data  
/odom – Odometry and ROV position  

Acknowledgments  
This project was developed as part of the bachelor thesis:  
“Autonomous ROV Platform for Efficient Underwater Sensor Testing”  
University of South-Eastern Norway – Campus Vestfold
in collaboration with Kongsberg Discovery.
