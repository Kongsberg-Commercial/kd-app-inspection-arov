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



### Image processor

### PID controller

### Odometry Publisher

### Ultrasonic Depth-sensor
