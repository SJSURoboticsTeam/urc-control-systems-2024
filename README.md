# urc-control-systems-2024
This Repository will host the code for the new SJSU Robotics Rover.
## To build:
```
conan build PATH -pr stm32f103c8  -pr arm-gcc-14.2 -b missing
```

Replace `PATH` with the corelating path to subsystem or driver(s). The folder contain a `conan.py` file. 

If you are already in in the folder run (for quick copy paste):

```
conan build . -pr stm32f103c8  -pr arm-gcc-14.2 -b missing
```
## To flash to controller:
```
stm32loader -e -w -v -B -p DEVICE BIN_PATH
```

If you are on mac or linux replace `DEVICE` with the path to the device you want to flash usually something like `/dev/tty.usbserial-10`

If you are on windows replace `DEVICE` with the com port of the device you want to flash usually something like `COM10`

Replace `BIN_PATH` with the path to the binary you wish the flash, from the build folder it the path is `build\stm32f103c8\MinSizeRel\application.elf.bin` for the main application. For demos there should be a `.elf.bin` file with a name matching the demo.

for more info refer to [libhal's getting started to the STM32](https://libhal.github.io/latest/getting_started/#uploading-demos-to-device)

If you are already in the folder and want run the main application (for quick copy paste, just replace ##):

**Unix:**
```
stm32loader -e -w -v -B -p /dev/tty.usbserial-## ./build/stm32f103c8/MinSizeRel/application.elf.bin
```

**Windows:** 
```
stm32loader -e -w -v -B -p COM## .\build\stm32f103c8\MinSizeRel\application.elf.bin
```