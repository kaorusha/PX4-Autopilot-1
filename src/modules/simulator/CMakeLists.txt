############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

option(ENABLE_UART_RC_INPUT "Enable RC Input from UART mavlink connection" OFF)

if(ENABLE_UART_RC_INPUT)
	if (APPLE)
		set(PIXHAWK_DEVICE "/dev/cu.usbmodem1")
	elseif (UNIX AND NOT APPLE)
		set(PIXHAWK_DEVICE "/dev/ttyACM0")
	elseif (WIN32)
		set(PIXHAWK_DEVICE "COM3")
	endif()

	set(PIXHAWK_DEVICE_BAUD 115200)
endif()
configure_file(simulator_config.h.in simulator_config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_add_module(
	MODULE modules__simulator
	MAIN simulator
	COMPILE_FLAGS
		-Wno-double-promotion
		-Wno-cast-align
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
	INCLUDES
		${CMAKE_BINARY_DIR}/mavlink
		${CMAKE_BINARY_DIR}/mavlink/development
	SRCS
		simulator.cpp
		simulator_mavlink.cpp
	DEPENDS
		mavlink_c_generate
		conversion
		geo
		drivers_accelerometer
		drivers_barometer
		drivers_gyroscope
		drivers_magnetometer
	)
target_include_directories(modules__simulator INTERFACE ${CMAKE_BINARY_DIR}/mavlink)

add_subdirectory(battery_simulator)
