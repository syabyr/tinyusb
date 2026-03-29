#*****************************************************************************
#
# CMakeLists.txt - Build configuration for USB2CAN example.
#
#*****************************************************************************

cmake_minimum_required(VERSION 3.13)

# Define the project
project(usb2can)

# Add source files
add_executable(usb2can
    usb2can.c
)

# Link libraries
target_link_libraries(usb2can PRIVATE tinyusb_device driverlib)
