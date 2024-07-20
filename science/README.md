# libhal-starter

Before getings started, if you haven't used libhal before, follow the
[Getting Started](https://libhal.github.io/2.1/) guide.

To build the project

```bash
conan build . -pr <target_name> -s build_type=<build_type>
```

For the `lpc4078` with `MinSizeRel`

```bash
conan build . -pr lpc4078 -s build_type=MinSizeRel
```

## Supported platforms

- lpc4078
- lpc4074

## Description of Files

1. `main.cpp`: The entry point of the application. It initializes the processor,
   platform, and starts the application. If any of these steps fail, it halts
   the system. If the application finishes, it either resets the resources or
   halts the system.
2. `application.cpp`: It contains the main loop of the application. In this
   example, it just prints "Hello, World" and toggles an LED on and off.
3. `application.hpp`: This file defines the `application_framework` struct which
   holds pointers to the LED, console, clock, and reset callback. Modify this
   structure to change the set of required drivers and settings for an
   application. It also declares the `initialize_processor`,
   `initialize_platform`, and `application` functions that must be implemented
   elsewhere.
4. `CMakeLists.txt`: This is the build file for the project. It sets the minimum
   required version of CMake, names the project, and sets the platform library.
   It also defines the sources to compile and the libraries to link against.
   Finally, it sets up the post-build steps with `libhal_post_build`.
5. `platforms/lpc4074.cpp`: This file provides implementations of the
   `initialize_processor` and `initialize_platform` functions for the LPC4074
   platform. In `initialize_platform`, it sets the clock speed, configures a
   UART for console output, and sets up an LED output pin. The returned
   `application_framework` has these peripherals, as well as a reset callback
   that resets the microcontroller.
6. `platforms/*.cpp`: Just like `platforms/lpc4074.cpp` but for any other
   platforms. It's important to note that the specifics of the
   `initialize_platform` function and the peripherals used will likely vary
   between different platforms. As an example the `lpc4074.cpp` file is
   specifically tailored to the LPC4074 microcontroller.

## Supporting Additional Platforms

Add the following section

```python
elif str(self.options.platform).startswith("<platform_name>"):
    self.requires("libhal-<platform_name>/[^2.0.0]")
```

into your `conanfile.py` (example: `libhal-stm32f1`):

```python
def requirements(self):
    self.requires("libhal-util/[^2.0.0]")

    if str(self.options.platform).startswith("lpc40"):
        self.requires("libhal-lpc40/[^2.0.0]")
    elif str(self.options.platform).startswith("stm32f1"):
        self.requires("libhal-stm32f1/[^2.0.0]")
    else:
        raise ConanInvalidConfiguration(
            f"The platform '{str(self.options.platform)}' is not supported.")
```

Add a new `<platform_name>.cpp` file in the `platform/` directory and implement
the `hal::status initialize_processor()` and
`hal::result<application_framework> initialize_platform()` functions based on
the platform. The platform's README.md should provide basic startup code for
each.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Disclaimer

This project is not an official Google project. It is not supported by
Google and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.
