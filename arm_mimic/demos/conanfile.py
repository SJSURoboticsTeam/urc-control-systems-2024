from conan import ConanFile


class demos(ConanFile):
    python_requires = "libhal-bootstrap/[^1.0.0]"
    python_requires_extend = "libhal-bootstrap.demo"

    def requirements(self):
        bootstrap = self.python_requires["libhal-bootstrap"]
        bootstrap.module.add_demo_requirements(self)
        