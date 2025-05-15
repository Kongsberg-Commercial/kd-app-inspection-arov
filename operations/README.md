This folder contains the ROV's code which is ran during different operations. Below is an overview of each script's role, how to run them, troubleshooting tips, and other considerations  


### Multilateration

There are two versions of the multilateration script. These are functionally the same, but **log_multilateration.py** will log the positioning data to some log files for post-mission analysis. Data from the multilateration algorithm is saved to `position_log.txt` while EKF data from MavLink is logged to `globalpos_log.txt`. 

Data is logged to sepperate files to avoid concurrency issues with file-descriptiors, as one thread may block another from writing to the logfile. 

##### Running 

These python scripts can be ran directly with the basic command:

```
python3 multilateration.py
```

##### Troubleshooting

If the cPAP does not receive any queries after a certain amount of time, it may enter sleep-mode to conserve battery. Usually, the cPAP will wake up when messages are sent through the serial port. However, there is a known issue where the cPAP is unable to wake itself up in rare occasions. To counteract this, it is reccomended to keep the multilateration script running during operational activity. If the cPAP does not wake up, retrieving the ROV and physically replugging the M8 cable connecter will usually work. 



### Image-processor

This script finds the angular error $\theta_e$ and lateral error $Y_e$ using OpenCV image processing on frames taken from the video feed. These are used by the pipefollow controller to correct the ROV's position.

##### Running

This script is located in `/home/pi/python/testing/mavlink_testing/` can be run using the command:

```
env/bin/python3 videoprocessor.py
```

Due to dependency issues with NumPy 2, this script has to be run using the python binary from the virtual environment folder. 

##### Troubleshooting
Ensure the correct python binary is used!
Ensure that the necessary ports / sockets are open, this can be checked in the main function of the code. 



### Pipefollow Controller

This script receives the angular error $\theta_e$ and lateral error $Y_e$ from the image-processor and controls the ROV using a PID-controller system. 

##### Running

The script can be found in `/home/pi/python/main` and can be run with the command:

```
python3 pipefollow_controller.py
```

### Odometry Publisher

### Collision Avoidance



### Cpap connection tester

This script can be useful to verify that connection with the cPAP is established before releasing the ROV into the water. 
This script simply checks the cPAP's battery. If the cPAP does not reply with a battery percentage, refer to the troubleshooting section

##### Running

The script can be run usint

```
python3 cpap_test.py
```

##### Troubleshooting
If no battery percentage is retrieved, ensure that the correct `/dev/ttyUSB` port is used. 
If the correct port is used, and no other scripts are connected to the serial port, then the cPAP may have entered sleep-mode. Replug the cPAP to wake it up.
