import serial
import time 

# Function to calculate the checksum of an NMEA message
def nmea_checksum(nmea_message):
    nmea_message = nmea_message.strip()
    assert nmea_message.startswith('$'), "[x] NMEA message must start with '$'"

    nmea_message = nmea_message[1:] # Remove trailing $
    data_list = nmea_message.split('*')
    assert len(data_list) == 2, "[x] FormatError, NMEA message must contain 1 and only 1 asterix (*)"
    data = data_list[0]

    checksum = 0
    for char in data:
        checksum ^= ord(char) # XOR with char-code of character
    
    return f"{checksum:02X}" # Format the checksum to hexadeciaml (capital)

# Helper function to send a message over serial and read the response.
def send_and_receive(ser, message):
    ser.write(message.encode('utf-8'))        # Write the message to the serial port.
    response = ser.readline().decode('utf-8', errors="ignore") # Read until we encounter newline. NMEA messages always end in <CR,LF>. Use ser.read_until(b'\r\n') if it doesnt work
    return response


# Updated functions that communicate with the cPAP using the serial port.
def read_battery_level(ser: serial.Serial):
    print(f"{60*'-'}\n[+] Read Battery Level")
    message = f"$PSIMCUP,BAT*"
    message += nmea_checksum(message)
    message += "\r\n"
    print(f"User -> cPAP command: {message.strip()}")
    reply = send_and_receive(ser, message)
    print(f"cPAP -> User reply:   {reply.strip()}")

def read_version_info(ser: serial.Serial):
    print(f"{60*'-'}\n[+] Read Version Info")
    message = f"$PSIMCUP,VER*"
    message += nmea_checksum(message)
    message += "\r\n"  # <CR,LF>
    print(f"User -> cPAP command: {message.strip()}")
    reply = send_and_receive(ser, message)
    print(f"cPAP -> User reply:   {reply.strip()}")

def set_transmit_power(ser: serial.Serial, level):
    print(f"{60*'-'}\n[+] Set Transmit Power to level {level}")
    assert level in [1, 2, 3, 4], "[x] FormatError, Transmit Power Level must be in [1,2,3,4]"
    message = f"$PSIMCUP,PWR,{level}*"
    message += nmea_checksum(message)
    message += "\r\n"  # <CR,LF>
    print(f"User -> cPAP command: {message.strip()}")
    reply = send_and_receive(ser, message)
    print(f"cPAP -> User reply:   {reply.strip()}")

def transponder_interrogation(ser: serial.Serial, channel: str, timeout: int):
    print(f"{60*'-'}\n[+] Transponder Interrogation with {channel}")
    assert type(timeout) in [int, float], "[x] TypeError, timeout must be of type int or float"
    assert 0 <= timeout <= 10, "[x] FormatError, Timeout must be in range (0s - 10s)"
    assert type(channel) == str, "[x] TypeError, channel must be of type str"
    assert channel in [f"M{i:02d}" for i in range(1, 57)], "[x] FormatError, channel must be in range (M01 - M56)"
    
    message = f"$PSIMCUP,TPI,{channel},{float(timeout)}*"
    message += nmea_checksum(message)
    message += "\r\n"  # <CR,LF>
    print(f"User -> cPAP command: {message.strip()}")
    reply = send_and_receive(ser, message)
    print(f"cPAP -> User reply:   {reply.strip()}")
    return reply

# Get the distance to a specific transponder. Calculates t_time * sound_speed. The variable sound_speed must be changed depending on air vs underwater testing
def get_distance(ser, channel, timeout):
    sound_speed = 1500 # Speed of sound measured in meters per second. 343m/s in air, 1481m/s underwater
    reply = transponder_interrogation(ser, channel, timeout)
    try:
        t_time = float(reply.split(",")[4])
        distance = t_time * sound_speed
        print(f"[+] Calculated distance to {channel}: {distance} meters")
    except:
        print(f"[x] No t_time in reply: {reply}")
        distance = None
    return distance

# Get the distacnes of multiple channels, and only returns if we received a reply from all of them
def get_multiple_distances(ser, channels, timeout):
    distances = []
    for channel in channels:
        distance = get_distance(ser, channel, timeout)
        if distance is not None:
            distances.append(distance)
    if(len(channels) == len(distances)):
        return distances
    print(f"[x] Error: Did not receive a reply from all transponders in {channels}")
    return None

if __name__ == "__main__":
    # Adjust 'baudrate', 'port', and other parameters as needed.
    # Baud rate is 9600 according to https://www.kongsberg.com/globalassets/kongsberg-discovery/commerce/navigation--positioning/cpap-subsea-transceiver/cpap-subsea-transceiver-instruction-manual/
    try:
        ser = serial.Serial(
            port='/dev/ttyUSB1',  # Change to RS232 adapter port (could be /dev/ttyS0, /dev/serial0, etc.)
            baudrate=9600,        # Set the baud rate expected by the cPAP.
            timeout=5
        )
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        exit(1)

    #set_transmit_power(ser,4)
    try:
        while True:
            read_battery_level(ser)
            time.sleep(1)
            #get_distance(ser, "M20", 4)
            #time.sleep(1)
    except KeyboardInterrupt:
        pass  # Gracefully exit when interrupted

    # Close the serial port when finished.
    ser.close()
