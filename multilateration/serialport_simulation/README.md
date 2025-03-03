This project folder consists of two scripts that attempt to simulate the serial communication with the cPAP, the movement of the ROV, and the multilateration algorithm to get the position

Interrogate Transponder -> Receive Transponder Interrogation Reply (t_time) -> Use distances to calculate ROV's position

##### cpap_part.py
The first script, cpap_part.py, is essentially what will be running on the raspberry pi on the ROV.
This script mixes the logic for the multilateration algorithm with the NMEA message construction and serial port communication.
We utilize a "background reader" that constantly listens to the serial port. The reply is handeled differently based on the NMEA message tags.
Using a "background reader" lets us send transponder interrogations without having to wait for the corresponding reply.
When all distance measurements from the transponder interrgations are received, we calculate the ROV's position using the multilateration algorithm.
Note that this simulation/emulator only uses a static depth-measurement (with some simulated error in the "sensor data"). In reality, we would like to use mavlink with real depth data.
Its also worth noting that the script currently only updates the positions if all four distance readings are received. This can be changed if the connections is still unstable, and we wish to use previous readings to get a less accurate estimate 

To start this script, simply run

```
python3 cpap_part.py
```

Note, make sure the serial port is correct! There are more details about simulating the serial port communication below

##### emulate_cpap_transponders.py
This script attempts to emulate the logic of the onboard cPAP and ROV.
We initialize a ROV object that moves in a circle. It's position is used to calculate the distance between the ROV and each transponder (with some added error in the "sensor-data") 
This script continuously listens on the serial port for interrogations or other NMEA commands, and based on the NMEA message tag, replies with the corresponding NMEA message reply.

The supported message types are the same as the cPAP:
- Transponder Interrogation
- Read battery level
- Read version info
- Set transmit power

To start this script, simply run

```
python3 emulate_cpap_transponders.py
```

This script should ideally be started before cpap_part.py, but it does not really matter
Note, make sure the serial port is correct! There are more details about simulating the serial port communication below

##### Simulating the serial port
To simulate serial port communication, we are using the linux command-line tool `socat`
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

This command will create a read and write port. 
```
2025/03/03 13:54:23 socat[446851] N PTY is /dev/pts/0
2025/03/03 13:54:23 socat[446851] N PTY is /dev/pts/2
2025/03/03 13:54:23 socat[446851] N starting data transfer loop with FDs [5,5] and [7,7]
```
Here, we would assign port 0 to cpap_part.py and port 2 to emulate_cpap_transponders.py


