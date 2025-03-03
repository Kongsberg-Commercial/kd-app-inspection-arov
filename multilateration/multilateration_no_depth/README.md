This script uses the multilateration algorithm without depth-readings.
This means we now have a dependency on one of the transponders being positioned deeper in the water than the others (the deeper, the better)
This also reduces the accuracy of the algorithm, hovering at around 1 meter of error 

This folder also includes the pure algorithm for testing the mathematics behind the algorithm using only distances (no depth reading)

Run multilateration_no_depth.py:
```
python3 multilateration_no_depth.py
```
