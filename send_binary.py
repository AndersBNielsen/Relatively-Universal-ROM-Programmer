import sys
import time
import serial

def send_file(serial_port, baud_rate, file_path, block_size, rom_pincount  ):
    try:
        # Open serial port
        ser = serial.Serial(serial_port, baud_rate, timeout=5, inter_byte_timeout=0.001, dsrdtr=True)
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
            ser.write(bytes([rom_pincount])) # ROM pin count
            block_count = 0
            error_count = 0
            transfer_started = False
            start_time = time.time()
            # Read the file and send in blocks
            while True:
                # Wait for the recipient's readiness signal (AA byte)
                byte = ser.read(1)
                
                if byte == b'\xAA':
                    if not transfer_started:
                        print("✓ Arduino ready - starting transfer...\n")
                        transfer_started = True

                    block_count += 1
                    data = f.read(block_size)
                    
                    if not data:
                        ser.write(b"\x00")    # send a zero-length block to indicate EOF to arduino
                        ser.flush()
                        elapsed = time.time() - start_time
                        rate = block_count / elapsed
                        print("")
                        print(f"Block {block_count} | {elapsed:.1f}s | {rate:.1f} blocks/s")
                        print(f"\n{'='*50}")
                        print(f"Transfer Summary:")
                        print(f"  Blocks sent: {block_count}")
                        print(f"  Time: {elapsed:.2f}s")
                        print(f"  Rate: {block_count/elapsed:.1f} blocks/s")
                        if error_count > 0:
                            print(f"  Errors: {error_count}")
                        print(f"{'='*50}")
                        break
                    
                    # Print block stats for first and then every 5 blocks
                    if block_count % 5 == 0 or block_count == 1:
                        elapsed = time.time() - start_time
                        rate = block_count / elapsed
                        if block_count != 1: print("")
                        print(f"Block {block_count} | {elapsed:.1f}s | {rate:.1f} blocks/s")
                        if block_count == 1: print(".", end="", flush=True)
                    else:
                        print(".", end="", flush=True)
                    
                    ser.write(data)
                    ser.flush()
                    time.sleep(0.02)  # 20ms delay to let Arduino process
                elif byte != b'\x00':   # surpress 0x00 bytes - caused as a result of D0/D1 being low during programming??
                    error_count += 1
                    print(f"\nWarning: Expected 0xAA, got 0x{byte.hex()} (error #{error_count})")
    except Exception as e:
        print("Error:", e)
    finally:
        # Close the serial port
        ser.close()

if __name__ == "__main__":
    # Check if correct number of command line arguments is provided
    if len(sys.argv) != 6:
        print("Usage: python send_binary.py <serial_port> <baud_rate> <file_path> <block_size> <rom_pincount>")
        sys.exit(1)

    # Extract command line arguments
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2])
    file_path = sys.argv[3]
    block_size = int(sys.argv[4])
    rom_pincount = int(sys.argv[5])  # Convert string to int

    # Call the function to send the file
    send_file(serial_port, baud_rate, file_path, block_size, rom_pincount)
