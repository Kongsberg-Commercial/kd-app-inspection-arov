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

## Project Structure  
/Assets  
  BlueRov2_underwaterscene.unity -> Main scene  
  /Scripts            -> C# scripts
  /StaticCamera       -> Script for the static camera  
  /Prefabs            -> Models and components  
  /Materials          -> Visual assets and effects  
  /Model              -> BlueROV2 model  
  /Pipes              -> Model of the pipe  
README.md  

Getting Started  
Clone the repository   
Open the project in Unity Hub.  
Open the main scene "BlueRov2_underwaterscene.unity". 
In the "ControlManager" gameobject in the hierarchy make sure the "Use Remote Control" is checked to enable MQTT data reception.  
In the "ROV-box" gameobject under "Smooth Mqtt Receiver" script, set the "Odom Y Offset" to 6 when navigating locally and 0.66 when navigating using the MQTT data.  
Make sure your MQTT broker is running and configured to accept telemetry data.  
In Unity, press Play to start the simulation.  

Guiding around in the 3D scene  
Use the wasd, arrows and qe on the keyboard to locally navigate around in the 3D scene.  
Press c on the keyboard in order to swicth views.  

MQTT Topics Used  
/pipe – Pipe-following controller data  
/odom – Odometry and ROV position  
/pipefollowspeed - Gived data to the digital twin in order to drive forward and follow the pipe  

Acknowledgments  
This project was developed as part of the bachelor thesis:  
“Autonomous ROV Platform for Efficient Underwater Sensor Testing”  
University of South-Eastern Norway – Campus Vestfold
in collaboration with Kongsberg Discovery.
