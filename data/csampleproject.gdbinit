#
# J-LINK GDB SERVER initialization
#
# This connects to a GDB Server listening
# for commands on localhost at tcp port 2331
target remote localhost:2331
# Reset the chip to get to a known state.
monitor reset
#
# CPU core initialization (to be done by user)
#
# Setup GDB FOR FASTER DOWNLOADS
# set remote memory-write-packet-size 1024
# set remote memory-write-packet-size fixed
# Load the program executable called "image.elf"
file ./Debug/app.elf
load
