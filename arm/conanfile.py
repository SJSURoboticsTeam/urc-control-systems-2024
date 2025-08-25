from conan import ConanFile

required_conan_version = ">=2.0.14"


class demos(ConanFile):
    python_requires = "libhal-bootstrap/[^4.1.0]"
    python_requires_extend = "libhal-bootstrap.demo"

    def requirements(self):
        bootstrap = self.python_requires["libhal-bootstrap"]
        # bootstrap.module.add_demo_requirements(self)
        self.requires("libhal-canrouter/[^3.0.0]")
        self.requires("libhal-arm-mcu/[latest]")
        self.requires("minimp3/cci.20211201")