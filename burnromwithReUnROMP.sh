#!/bin/bash
## Assuming this is run from the root of the 65uino repo with Programmer as a submodule
python Programmer/send_command.py /dev/cu.usbserial-* 19200 03 DA 08  # 03 = erase, DA = Winbond, 08 = W27C512
# After sending data there's a timeout before loading happens, so we need a delay
sleep 2 
python Programmer/send_binary.py /dev/cu.usbserial-* 19200 build/abn6507rom.bin 32 
sleep 2
#Read back the ROM we just programmed to check if it's good  
python Programmer/read_binary.py /dev/cu.usbserial-* 19200 verify.bin 4096 32
diff -s build/abn6507rom.bin verify.bin