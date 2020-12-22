*** GCC COMPILE INSTRUCTIONS ***

Prerequisites
- apt install gcc-arm-none-eabi
- a python 2.7 environment
- install prerequisites from requirements.txt (pip install -r requirements.txt)

Building
- Activate the python environment.
- Navigate to folder with Makefile
- "make" to build.
- "make clean" to clean.


Debugging
- launch openocd server
-- sudo openocd -f openocd.cfg 
- attach to server from the build directory after compile
-- arm-none-eabi-gdb -tui
--- target remote:3333
--- file stm32h743ii_if.elf 
--- mon reset halt
--- load
--- continue


