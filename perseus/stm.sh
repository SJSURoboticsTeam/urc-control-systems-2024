// ...existing code...
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# First argument is the binary filename (e.g. pid_position_test.elf.bin).
# Defaults to velocity_test.elf.bin if not provided.
bin="${1:-velocity_test}"

echo "Building with Conan..."
conan build . -pr stm32f103c8 -pr arm-gcc-12.3 -b missing

echo "Flashing with stm32loader..."
stm32loader -e -w -v -B -p /dev/ttyACM0 "build/stm32f103c8/MinSizeRel/${bin}.elf.bin"
