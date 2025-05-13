官方用户指南[PX4 Autopilot User Guide | PX4 Guide (main)](https://docs.px4.io/main/en/)

# Ubuntu22.04

### 启动Gazebo SITL

```shell
make px4_sitl gazebo-classic
```

### 上传固件（刷写板）

```shell
make px4_fmu-v4_default upload
```

### 简单地示例代码

```c++
/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/*订阅传感器主题 */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* 初始化要发布的主题并公布主题 */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
```

### 事件接口

```c++
#include <px4_platform_common/events.h>
```

```c++
events::send(events::ID("event_name"), events::Log::Info, "Test Message");
```

最小示例

```c++
uint8_t arg1 = 0;
float arg2 = -1.f;
/* EVENT
 * @description
 * This is the detailed event description.
 * - value of arg1: {1}
 * - value of arg2: {2:.1}
 */
events::send<uint8_t, float>(events::ID("event_name"),
	{events::Log::Error, events::LogInternal::Info}, "Event Message", arg1, arg2);
```

## 飞行任务

### 任务创建

#### 1.新建任务文件夹

```shell
mkdir PX4-Autopilot/src/modules/flight_mode_manager/tasks/MyTask
```

MyTask:按照需求自己设定名字

#### 2.在 *MyTask* 目录中使用前缀 “FlightTask” 为新的飞行任务创建空源代码和 *cmake* 文件：

- CMakeLists.txt
- FlightTaskMyTask.hpp
- FlightTaskMyTask.cpp

```shell
gedit FlightTaskMyTask.hpp
```

把以下内容写入CMakeLists.txt，此代码来源于 Orbit/CMakeLists.txt

```c++
############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

px4_add_library(FlightTaskOrbit
	FlightTaskOrbit.cpp
)

target_link_libraries(FlightTaskOrbit PUBLIC FlightTaskManualAltitudeSmoothVel SlewRate)
target_include_directories(FlightTaskOrbit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
```

修改代码表示新的任务

```c++
px4_add_library(FlightTaskMyTask
    FlightTaskMyTask.cpp
)

target_link_libraries(FlightTaskMyTask PUBLIC FlightTask)
target_include_directories(FlightTaskMyTask PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
```

修改hpp文件如下

```c++
#pragma once

#include "FlightTask.hpp"

class FlightTaskMyTask : public FlightTask
{
public:
  FlightTaskMyTask() = default;
  virtual ~FlightTaskMyTask() = default;

  bool update();
  bool activate(const trajectory_setpoint_s &last_setpoint) override;

private:
  float _origin_z{0.f};
};
```

根据需要更新 cpp 文件。此示例提供了 **FlightTaskMyTask.cpp** 的简单实现，它仅指示调用了 task 方法

```c++
#include "FlightTaskMyTask.hpp"

bool FlightTaskMyTask::activate(const trajectory_setpoint_s &last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);
  PX4_INFO("FlightTaskMyTask activate was called! ret: %d", ret); // report if activation was successful
  return ret;
}

bool FlightTaskMyTask::update()
{
  PX4_INFO("FlightTaskMyTask update was called!"); // report update
  return true;
}
```

#### 3.将新任务添加到要在 [PX4-Autopilot/src/modules/flight_mode_manager/CMakeLists.txt](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/CMakeLists.txt#L41) 中构建的任务列表中。

```c++
 list(APPEND flight_tasks_all
   Auto
   Descend
   ...
   ManualPositionSmoothVel
   Transition
   {MyTask}//根据自己任务的名字更改
 )
```

#### 4.更新 Flight Mode 以确保调用任务。通常使用参数来选择何时应使用特定的飞行任务。

若为多旋翼 Position 模式 则找到src/modules/mc_pos_control/multicopter_position_mode_params.c

在其中增加一个选项，如果参数具有以前未使用的值（如 5），则用于选择 “MyTask”

```c++
...
 * @value 0 Direct velocity
 * @value 3 Smoothed velocity
 * @value 4 Acceleration based
 * @value 5 My task
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_POS_MODE, 5);
```

#### 5.在开关中为参数 [FlightModeManager.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/flight_mode_manager/FlightModeManager.cpp#L266-L285) 为新选项添加一个 case，以启用任务 when 具有正确的值。`_param_mpc_pos_mode`

```c++
switch (_param_mpc_pos_mode.get()) {
  ...
  case 3:
     error = switchTask(FlightTaskIndex::ManualPositionSmoothVel);
     break;
  case 5: // Add case for new task: MyTask
     error = switchTask(FlightTaskIndex::MyTask);
     break;
  case 4:
```

#### 6.测试任务

构建 SITL 模拟 （gazebo-classic）

```c++
make px4_sitl gazebo-classic
```

打开 QGroundControl 

在控制台中，起飞并切换到 Position 模式：

```c++
pxh> commander takeoff
pxh> commander mode posctl
```

# Matlab:Simulink

