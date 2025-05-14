This folder contains the ROV's code which is ran during different operations. Below is an overview of each script's role, how to run them, troubleshooting tips, and other considerations  


### Multilateration

There are two versions of the multilateration script. These are functionally the same, but **log_multilateration.py** will log the positioning data to some log files for post-mission analysis. Data from the multilateration algorithm is saved to `position_log.txt` while EKF data from MavLink is logged to `globalpos_log.txt`. 

Data is logged to sepperate files to avoid concurrency issues with file-descriptiors, as one thread may block another from writing to the logfile. 


### Image processor

### PID controller

### Odometry Publisher

### Ultrasonic Depth-sensor
