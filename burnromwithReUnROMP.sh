#!/bin/bash
python3 send_command.py /dev/cu.usbserial-* 9600 03 DA 08  # 03 = erase, DA = Winbond, 08 = W27C512
# After sending data there's a timeout before loading happens, so we need a delay
sleep 2 
python3 send_binary.py /dev/cu.usbserial-* 9600 build/abn6507rom.bin 32 
sleep 2
#Read back the ROM we just programmed to check if it's good  
python3 read_binary.py /dev/cu.usbserial-* 9600 verify.bin 4096 32 
diff -s build/abn6507rom.bin verify.bin