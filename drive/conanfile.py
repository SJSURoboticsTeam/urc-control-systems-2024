from conan import ConanFile

required_conan_version = ">=2.0.14"


# class demos(ConanFile):
#     python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
#     python_requires_extend = "libhal-bootstrap.demo"

#     def requirements(self):
#         bootstrap = self.python_requires["libhal-bootstrap"]
#         bootstrap.module.add_demo_requirements(self)
#         self.requires("libhal-canrouter/[^3.0.0]")
#         self.requires("libhal-actuator/[1.2.2 || latest]")
#         # self.requires("libhal-util/[^5.4.0]")
#         # self.requires("libhal-arm-mcu/[1.15.0 || latest]")
#         self.requires("minimp3/cci.20211201")
class demos(ConanFile):
    python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
    python_requires_extend = "libhal-bootstrap.demo"

    def requirements(self):
        self.requires("libhal-util/[^5.4.0]")
        self.requires("libhal-arm-mcu/1.18.0")
        self.requires("minimp3/cci.20211201")
        self.requires("libhal-actuator/[1.2.2 || latest]")