# Relatively Universal-ROM-Programmer
 
This is meant to be a cheap way to program W27C512 EEPROMs which require 14V erasure voltage and 12V programming voltage... Or basically any other 24/28/32 pin ROM requiring from 5V to 27V programming/erasure voltage. 

You can either make your own, get a PCB from JLCPCB using the guide below or order a kit with SMD components presoldered from [iMania.dk](https://www.imania.dk/index.php?currency=EUR&cPath=204&sort=5a&language=en) (sockets available if needed).
Kits with SMD components presoldered are available from my store, [iMania.dk](https://www.imania.dk/index.php?currency=EUR&cPath=204&sort=5a&language=en)

Requires a dev board with a processor or microcontroller. Like a [65uino](https://github.com/AndersBNielsen/65uino) or Arduino Uno for instance. 

## Software

#1 Don't attach the Programmer shield while power is on.

#2 Don't turn on power if the controller (Arduino/65uino/Nucleo board) isn't programmed with appropriate firmware. 

#3 Don't insert a ROM in the socket until you're ready to write it. Don't leave a ROM in the socket during reset or programming. 

### Firestarter firmware
If you just want to get started fast, there's an amazing third party firmware and desktop software available in the form of Henols's [Firestarter](https://github.com/henols/firestarter)
Firestarter is recommended if you just want to program ROMs cheaply, easily, and don't care too much about the technical details. 

Remember: Don't insert a ROM in the socket until you're ready to write it. Don't leave a ROM in the socket during reset or programming. Program Arduino before attaching shield. Attach shield before powering on Arduino. 

### Original proof of concept code

6502 assembly code is available in the [65uino repo](https://github.com/AndersBNielsen/65uino) (to be separated and moved here "soon"). 

Arduino sketch is available in this repo's [/software](https://github.com/AndersBNielsen/Relatively-Universal-ROM-Programmer/tree/main/software/Arduino/ArduinoProgrammerFirmwarePrototype)

Both the 6502 firmware (65uino) and the Arduino sketch use the command line python scripts in this repo for burning and reading ROMs. 

read_binary.py reads a ROM via serial. 

send_binary.py burns a ROM via serial.

send_command.py issues commands (erase for instance)


## Rev 0 photos (The Rev 1 added a voltage divider for reading VPP/VEP, Rev2 has replaced the jumpers with transistors for automatic pin count selection)

![Relatively-Universal-ROM-programmer-rev0-front-assembled](https://github.com/AndersBNielsen/Relatively-Universal-ROM-Programmer/assets/7676834/0a7f2735-8641-42c2-a1d2-de253550fb94)
![Romprogrammer-assembled-back](https://github.com/AndersBNielsen/Relatively-Universal-ROM-Programmer/assets/7676834/a7d42e57-f92c-425c-8056-8d52cf872a45)
![ROMprogrammer-assembled](https://github.com/AndersBNielsen/Relatively-Universal-ROM-Programmer/assets/7676834/76f9c717-3eef-4829-b960-7c5585a4fdbc)

## Documentation
The Relatively Universal ROM Programmer relies on the standard JEDEC ROM footprint and common "high voltage" pins to work. BJT drivers handle putting the 12-27V programming voltage on the relevant pins. Which "high voltage" pins are active can be selected in software and BJT drivers control which pins can be used as VCC. 
The 12-27V for programming is manually set using a small screwdriver and a trimpot with a readout in software or multimeter if your micro doesn't have analog pins (65uino).

## Jumpers

### Revision 2.X
Revision 2 and later doesn't need physical jumpers to select 24, 28 and 32 pin ROMs, as it has transistor drivers to pass either VCC or the appropriate address line to the correct physical pin.

Revision 2 and 2.1 *do* however have a jumper to connect high voltage to pin 1 of 28 pin ROMs (JP4) - do *NOT* leave this jumper in place when programming 32 pin ROMs as it will most likely damage the ROM, the programmer, or both. 

<img width="1334" alt="JP4" src="https://github.com/user-attachments/assets/7122f735-b504-4849-a045-24fc4247cd34" />

Revision 2.2 also has JP4 but rotated 90 degrees (green) and another option of powering the VPP pin of 2716-type ROMs (red). 
Again: *DO NOT* leave JP4 in place in either rotation if you are not programming 28 pin ROMs with VPP on pin 1 (check the ROM's datasheet or "firestarter info") or 2716 type ROMs with a very nonstandard VPP pin. 


<img width="761" alt="image" src="https://github.com/user-attachments/assets/caa09fc9-7daf-441d-b80b-a88764f96f02" />

### Revision 1.x
Revision 1.x require physical jumpers to connect VCC to 24 and 28 pin ROMs

A common configuration is to connect "A" to 5V and B to "A13", which covers the Winbond 27C512 and many other common 28 pin ROMs. This configuration also covers the 32 pin SST39SF010 (5V programmable).

Documentation is on the back of Revision 1.x boards
<img width="667" alt="image" src="https://github.com/user-attachments/assets/d8ce287d-f66f-4936-98dc-9dbbb9253c3d" />


### Setting the voltage 

The "high voltage", VPE, is calibrated using the trimpot on the front of the board after activating the regulator in software. Please be aware that some dev boards WILL put VPE on several pins during reset - especially if reset is held down manually. This MAY damage a chip in the socket (especially if it's only 5V programmable). 
Clarity: It's best to NOT reset the controller with a ROM in the socket to be on the safe side. 

Documentation will be updated with FAQ.

![Programmer-back-documentation](https://github.com/AndersBNielsen/Relatively-Universal-ROM-Programmer/assets/7676834/4f94149c-8b54-4666-a778-0e467afc1d95)

## Getting a PCB
This project is kindly sponsored by JLCPCB. They offer cheap, professional looking PCBs and super fast delivery.

Step 1: Get the gerber file zip package from the /hardware folder
/hardware/rev2/rev2-1316.zip for instance.

Step 2: Upload to JLCPCB [https://jlcpcb.com](https://jlcpcb.com/?from=Anders_N)

<img src="https://github.com/AndersBNielsen/65uino/blob/main/images/upload.png?raw=true" alt="Upload" style="width: 220px;">

Step 3: Pick your color, surface finish and order.

<img src="https://github.com/AndersBNielsen/65uino/blob/main/images/settings.png?raw=true" alt="Select settings" style="width: 220px;">

<img src="https://github.com/AndersBNielsen/65uino/blob/main/images/save.png?raw=true" alt="Save your choice" style="width: 220px;">

If you want JLCPCB to assemble a board, the BOM and placement files are also in the repo. 

You can use these affiliate links to get a board for $2 and also get $80 worth of New User Coupons at: https://jlcpcb.com/?from=Anders_N

And in case you also want to order a 3D-printed case you can use this link. 
How to Get a $7 3D Printing Coupon: https://jlc3dp.com/?from=Anders_N
