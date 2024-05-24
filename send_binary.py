import sys
import time
import serial

def send_file(serial_port, baud_rate, file_path, block_size):
    try:
        # Open serial port
        ser = serial.Serial(serial_port, baud_rate, timeout = 0.001, inter_byte_timeout=0.001, dsrdtr=True)
        # Disable the DTR signal to prevent reset
        ser.dtr = False
        time.sleep(2)
        
        start_time = time.time()
        
        # Open the file to send
        with open(file_path, 'rb') as f:
            # Send block size as a byte
            ser.write(b'\xAA') # Command mode
            ser.write(b'\x02') # Burn ROM command
            ser.write(bytes([block_size]))
            ser.write(b'\x10') # Stop page.. Fix

            # Read the file and send in blocks
            while True:
                # Wait for the recipient's readiness signal (AA byte)
                byte = ser.read(1)
                if byte == b'\xAA':
                    # Read the next block of data
                    data = f.read(block_size)
                    if not data:
                        break
                    # Send the data block
                    ser.write(data)
                    #print(".", end="")
                # Record end time
                else:
                    pass


        end_time = time.time()
        # Calculate total duration
        total_duration = end_time - start_time
        print(f"File sent in {total_duration:.2f} seconds")

        print("\nFile sent successfully!")

    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Check if correct number of command line arguments is provided
    if len(sys.argv) != 5:
        print("Usage: python send_binary.py <serial_port> <baud_rate> <file_path> <block_size>")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]
    block_size = int(sys.argv[4])

    # Call the function to send the file
    send_file(serial_port, baud_rate, file_path, block_size)
