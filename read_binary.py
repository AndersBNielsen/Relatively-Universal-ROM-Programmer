import sys
import time
import serial

def get_file(serial_port, baud_rate, file_path, total_bytes, block_size):
    try:
        # Open serial port
        ser = serial.Serial(serial_port, baud_rate)
        time.sleep(4)

        ser.write(b'\xAA') # Command byte
        # Send block size as a byte
        ser.write(b'\x01') # Command
        ser.write(bytes([block_size])) # Block size
        ser.write(b'\x00') #LSB
        ser.write(b'\x00') #MSB
        stoppage = (total_bytes >> 8) & 0xFF 
        ser.write(bytes([stoppage])) # Stoppage

        start_time = time.time()

        # Initialize a buffer to store received data
        buffer = bytearray()

        try:
            bytes_received = 0
            while True:
                if (ser.inWaiting() > 0):
                    # Read one byte with timeout
                    byte = ser.read(1)

                    # Check if start of frame is received
                    if byte == b'\xAA':
                        #print(".", end="")
                        # Read specified block size bytes
                        frame = ser.read(block_size)
                        buffer.extend(frame)
                        bytes_received += len(frame)  # Update total bytes received
                    
                    if bytes_received >= total_bytes:
                        break

            end_time = time.time()
            # Calculate total duration
            total_duration = end_time - start_time
            print(f"File received in {total_duration:.2f} seconds")

        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            print("\nSerial reading stopped by user at ")
            print(bytes_received)
        
        # Write buffer to file
        with open(file_path, 'wb') as f:
            f.write(buffer)

    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Check if correct number of command line arguments is provided
    if len(sys.argv) != 6:
        print("Usage: python read_binary.py <serial_port> <baud_rate> <file_path> <total_bytes> <block_size>")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]
    total_bytes = int(sys.argv[4])
    block_size = int(sys.argv[5])

    # Call the function to send the file
    get_file(serial_port, baud_rate, file_path, total_bytes, block_size)
