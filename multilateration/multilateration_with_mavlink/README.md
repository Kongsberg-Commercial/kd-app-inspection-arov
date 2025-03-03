This is an attempt at integrating pymavlink into the multilateration script.
We instansiate a seprerate thread for the pymavlink reader, which listens on `udpin:0.0.0.0:7777`
Make sure the UDP client on 127.0.0.1 port 7777 is enabled in the Mavlink Endpoint section on BlueOS.
To get the depth reading, we are listening on the message `VFR_HUD` (https://mavlink.io/en/messages/common.html#VFR_HUD)
The depth reading is available at VFR_HUD.alt (altitude). This is measured in meters. 
This reading is stored in a global variable which is retrieved by a getter function that defaults to a safe -6 meter reading if the message isnt available (essentially the same as the static reading)

Alternatively, we can use the SCALED_PRESSURE or SCALED_PRESSURE2 messages to get pressure and temperature, which can be used to calculate the depth using the code linked by Karl Thomas on Teams.

Note that this script has not been tested due to the dependency on receiving data through pymavlink. 
Please consult chatgpt if issues related to the pymavlink connection occur.