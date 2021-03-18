# Zephyr serial loader subsystem 
This module is a subsystem of the Zephyr shell that allows data to be loaded into device memory via UART.
After using a command in the Zephyr shell, the device reads all transferred UART data and writes it in 4 byte words to an address in the memory. The transfer ends when the user presses `ctrl+d`.

## Requirements
* Zephyr RTOS with shell subsystem enabled
* A board with UART polling support, for instance [Quicklogic Quickfeather board](https://github.com/QuickLogic-Corp/quick-feather-dev-board)

## Building

The sample can be build for several platforms, the following commands build the application with a shell for the QuickFeather board.
```bash
west build -b quick_feather samples/subsys/serial_loader
```
See [QuickFeather programming and debugging](https://docs.zephyrproject.org/latest/boards/arm/quick_feather/doc/index.html#programming-and-debugging) on how to run an image to the board.

## Running
After connecting to the UART console you should see the following output:
```bash
*** Booting Zephyr OS build v2.5.0-rc1-125-g25a1c4394db9  ***

uart:~$  
```
The serial loader command can now be used (`serial load [option] [address]`):
```bash
uart:~$ serial load 0x10000
Loading...
Press ctrl+d to stop
```

Now, the serial loader is waiting for data. You can either type it directly from the console or send it from the host PC (replace `ttyX` with the appropriate one for your UART console):
```bash
xxd -p data > /dev/ttyX
```
(It is important to use plain-style hex dump)
Once the data is transferred, use `ctrl+d` to quit loader. It will print the sum of the read bytes and return to the shell:
```bash
sum of bytes read: 75960
uart:~$
```

## Options
Currently, the serial loader supports the option:
* `-e` little endian parse e.g. `0xDEADBEFF -> 0xFFBEADDE`
