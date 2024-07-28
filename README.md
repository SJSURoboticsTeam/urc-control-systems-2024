# urc-control-systems-2024
This Repository will host the code for the new SJSU Robotics Rover.
To build: 
To build the project:
### Note: the LPC4078 and stm32f1 platforms are not included, simply the micromod in the arm subsystem. 
```bash
conan build . -pr <target_name> -pr <compiler>
```
For the `lpc4078 micro mod`:

For the `lpc4078`
```bash
conan build . -pr mod-lpc40-v5 -pr arm-gcc-12.3
```
```bash
conan build . -pr lpc4078 -pr arm-gcc-12.3
```


For the STM32F103 MicroMod V4:

```bash
conan build . -pr mod-stm32f1-v4 -pr arm-gcc-12.3
```

## Installing Platform Profiles

`lpc40` profiles:

```bash
conan config install -sf conan/profiles/v2 -tf profiles https://github.com/libhal/libhal-lpc40.git
```

`stm32f1` profiles:

```bash
conan config install -sf conan/profiles/v2 -tf profiles https://github.com/libhal/libhal-stm32f1.git
```

`micromod` profiles:

```bash
conan config install -sf conan/profiles/v1 -tf profiles https://github.com/libhal/libhal-micromod.git
```
