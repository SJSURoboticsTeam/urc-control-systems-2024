from conan import ConanFile

required_conan_version = ">=2.0.14"

class demos(ConanFile):
    python_requires = "libhal-bootstrap/[4.3.0]"
    python_requires_extend = "libhal-bootstrap.demo"

    def requirements(self):
        self.requires("libhal/4.18.0")
        self.requires("libhal-util/[5.7.0]")
        self.requires("libhal-arm-mcu/1.18.0")
        self.requires("libhal-actuator/[1.2.3]")
