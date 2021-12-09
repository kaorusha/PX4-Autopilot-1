/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "IgnitionSimulator.hpp"

#include <iostream>
#include <string>

void IgnitionSimulator::ImuCallback(const ignition::msgs::IMU &imu)
{
	// FLU -> FRD
	_px4_accel.update(hrt_absolute_time(),
			  imu.linear_acceleration().x(),
			  -imu.linear_acceleration().y(),
			  -imu.linear_acceleration().z());

	_px4_gyro.update(hrt_absolute_time(),
			 imu.angular_velocity().x(),
			 -imu.angular_velocity().y(),
			 -imu.angular_velocity().z());
}

void IgnitionSimulator::PoseInfoCallback(const ignition::msgs::Pose_V &pose)
{
	// TODO: find by name or id?
	//

	//pose.position
	//pose.orientation


	// TODO:
	//  - gps
	//  - magnetometer
	//  - barometer

	//  - groundtruth
}

IgnitionSimulator::IgnitionSimulator() :
	ModuleParams(nullptr),
	_px4_accel(1310988), // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	_px4_gyro(1310988)   // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
{
	_px4_accel.set_range(2000.f); // don't care
	_px4_gyro.set_scale(math::radians(2000.f) / static_cast<float>(INT16_MAX - 1)); // 2000 degrees/second max
}

void IgnitionSimulator::run()
{
	ignition::transport::Node node;

	// Subscribe to messages of other plugins.
	node.Subscribe("/imu", &IgnitionSimulator::ImuCallback, this);
	node.Subscribe("/world/quadcopter/pose/info", &IgnitionSimulator::PoseInfoCallback, this);



	// TODO: publish


	// Zzzzzz.
	ignition::transport::waitForShutdown();
}

int IgnitionSimulator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

IgnitionSimulator *IgnitionSimulator::instantiate(int argc, char *argv[])
{
	IgnitionSimulator *instance = new IgnitionSimulator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int IgnitionSimulator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int IgnitionSimulator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IgnitionSimulator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ignition_simulator", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ignition_simulator_main(int argc, char *argv[])
{
	return IgnitionSimulator::main(argc, argv);
}
