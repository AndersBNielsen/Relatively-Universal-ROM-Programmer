import sys
import time
import serial

def send_file(serial_port, baud_rate, command, opt1, opt2):
    try:
        # Open serial port
        ser = serial.Serial(serial_port, baud_rate, timeout=0)
        # Send Start of Header (SOH) byte (hex 0x01)
        time.sleep(3)
        ser.write(b'\xAA')
        ser.write(bytes.fromhex(command))

        if opt1:
            ser.write(bytes.fromhex(opt1))
        if opt2:
            ser.write(bytes.fromhex(opt2))

        time.sleep(0.1) #Closing too fast empties TX buffer without sending 

        print("Command sent successfully.")

    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Check if correct number of command line arguments is provided
    if len(sys.argv) < 4:
        print("Usage: python send_userland.py <serial_port> <baud_rate> <command in hex> <optional parameters>")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    command = sys.argv[3]
    opt1       = sys.argv[4]
    opt2       = sys.argv[5]

    # Call the function to send the file
    send_file(serial_port, baud_rate, command, opt1,opt2)
