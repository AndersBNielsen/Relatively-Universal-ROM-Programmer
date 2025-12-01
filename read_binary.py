import sys
import time
import serial

def get_file(serial_port, baud_rate, file_path, total_bytes, block_size, rom_pincount, nonstandard_pinout):
    try:
        # Open serial port
        print("Open Serial port and Sleep 4s")
        ser = serial.Serial(serial_port, baud_rate)
        time.sleep(4)
        print("Sleep Done")
        ser.write(b'\xAA') # Command byte
        # Send block size as a byte
        ser.write(b'\x01') # Command
        ser.write(bytes([block_size])) # Block size
        ser.write(b'\x00') #LSB
        ser.write(b'\x00') #MSB
        stoppage = (total_bytes >> 8) & 0xFF
        ser.write(bytes([stoppage])) # Stoppage
        ser.write(bytes([rom_pincount])) # ROM pin count
        # if nonstandard_pinout > 0  pincount is 24 and total_bytes 4096
        # it might be a TMS2532 type, check options and write 1 or 0 or do nothing
        if (nonstandard_pinout > 0) :
            ser.write(b'\x01')
            print("DEBUG: NON_STANDARD_SENT")
            # It seems that the previous (not my version) of arduino code
            # doesn't notice or care about the extra byte
        print("Starting Read")
        start_time = time.time()

        # Initialize a buffer to store received data
        buffer = bytearray()

        try:
            bytes_received = 0
            print("Begin while True")
            while True:
                
                if (ser.inWaiting() > 0):
                    # Read one byte with timeout
                    byte = ser.read(1)
                    #print("B")

                    # Check if start of frame is received
                    if byte == b'\xAA':
                        # print(".", end="")
                        # Read specified block size bytes
                        frame = ser.read(block_size)
                        buffer.extend(frame)
                        bytes_received += len(frame)  # Update total bytes received
                        # print(f"Received { bytes_received } of { total_bytes } bytes", flush=True)
                        print("F",end="", flush=True)
                    
                    if bytes_received >= total_bytes:
                        print("")
                        print("DONE", flush=True)
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
    if ( len(sys.argv) < 7) or ( len(sys.argv) > 8 ) :
        print("Usage: python read_binary.py <serial_port> <baud_rate> <file_path> <total_bytes> <block_size> <rom pincount> [nonstandard_pinout STRING]")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]
    total_bytes = int(sys.argv[4])
    block_size = int(sys.argv[5])
    rom_pincount = int(sys.argv[6])  # Convert string to int
    
    # Test valid nonstandard_pinout strings here
    # "TMS2532" if invalid, then exit
    # FIXME
    if len(sys.argv) != 8 :
        nonstandard_pinout = 0;
    if len(sys.argv) == 8 :
        match sys.argv[7] :
            case "TMS2532":
               nonstandard_pinout = 1
            # add new cases here
            case _:
               print("Invalid nonstandard_pinout STRING")
               sys.exit(1)

    # Call the function to send the file
    get_file(serial_port, baud_rate, file_path, total_bytes, block_size, rom_pincount, nonstandard_pinout)
