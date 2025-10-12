

## 一、创建工作空间

```shell
创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
 
编译工作空间
catkin_make
 
设置环境变量
source devel/setup.bash
 
检查环境变量
echo $ROS_PACKAGE_PATH
 
创建功能包
catkin_create_pkg  rospy roscpp std_msgs
 
编译功能包
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
 
编译制定功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES="包名"
 
```

## 二、话题编程

话题编程流程

1. 创建发布者
2. 创建订阅者
3. 添加编译选项
4. 运行可执行文件

```shell
/**
 * 该例程将订阅chatter话题，消息类型String
 * listener.cpp
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"
 
// 接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
 
int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "listener");
 
  // 创建节点句柄
  ros::NodeHandle n;
 
  // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
 
  // 循环等待回调函数
  ros::spin();
 
  return 0;
}
```

### 如何编译代码

1. 设置需要编译的代码和生成的可执行文件
2. 设置链接库
3. 设置依赖

```shell
在CMakeLists.txt中加入
 
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)
 
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)
```

### 运行代码

```shell
1.启动roscore
 
2.设置环境变量
source ~/catkin_ws/devel/setup.bash
 
3.运行发布者
rosrun learning talker
 
4.运行订阅者
rosrun learning listener
```

### **自定义msg话题消息**

```shell
话题person.msg数据定义
 
string name
uint8 sex
uint8 age
 
uint8 unknown=0
uint8 male=1
uint8 female=2

```

```shell
1.定义msg文件
 
2.在package.xml中添加功能包依赖
 <build_depend>message_generation</build_depend>
 <exec_depend>message_runtime</exec_depend>
 
3.在CMakeLists.txt中添加编译选项
  find_package( …… message_generation)
 
  catkin_package(CATKIN_DEPENDS geometry_msgs roscpp
  rospy std_msgs message_runtime)
 
  add_message_files(FILES Person.msg)
  generate_messages(DEPENDENCIES std_msgs)
 
 
注：部分ros版本中的exec_depend需要改成run_depend
```

## 三、服务-客户端

在 ROS（Robot Operating System）中，**服务（Service）** 和**客户端（Client）** 是一种基于**请求 - 响应（Request-Response）** 模式的通信机制，适用于需要**双向交互**（节点 A 向节点 B 发送请求，节点 B 处理后返回结果）的场景，与话题（Topic）的单向异步通信形成互补。

### 1.核心概念

1. **服务（Service）**由一个节点提供，用于处理其他节点的请求并返回响应。服务需要定义**请求数据结构**和**响应数据结构**，形成一个服务类型（`.srv` 文件）。
2. **客户端（Client）**由一个节点创建，用于向服务端发送请求，并等待服务端返回响应。客户端需要知道服务的名称和类型，才能正确通信。

### 2.服务的定义（`.srv` 文件）

服务类型通过 `.srv` 文件定义，放在功能包的 `srv` 目录下，格式为：

```tex
请求数据类型  请求字段名
---
响应数据类型  响应字段名
```

For example:

```tex
int64 a
int64 b
---
int64 sum
```

- 请求：两个整数 `a` 和 `b`
- 响应：它们的和 `sum`

### 3.工作流程

1. **服务端（Provider）**：
   - 定义服务回调函数（处理请求并生成响应）。
   - 向 ROS 主节点（`roscore`）注册服务，声明服务名称和类型。
2. **客户端（Requester）**：
   - 向 ROS 主节点查询指定名称和类型的服务。
   - 找到服务后，发送请求数据。
   - 等待服务端处理并返回响应。
3. **通信过程**：客户端发送请求 → 服务端接收并处理 → 服务端返回响应 → 客户端接收响应。（整个过程是**同步**的，客户端会阻塞等待响应）

### 4.C++ 实现示例

#### 1. 服务端代码（提供加法服务）

```c++
#include "ros/ros.h"
#include "my_package/AddTwoInts.h"  // 自定义服务类型头文件

// 服务回调函数：处理请求，生成响应
bool add(my_package::AddTwoInts::Request  &req,
         my_package::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;  // 计算求和结果
  ROS_INFO("收到请求: a=%ld, b=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("返回响应: sum=%ld", (long int)res.sum);
  return true;  // 表示处理成功
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");  // 初始化节点
  ros::NodeHandle n;

  // 注册服务，服务名称为"add_two_ints"，回调函数为add
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("加法服务已启动，等待请求...");

  ros::spin();  // 进入事件循环，等待请求

  return 0;
}
```

#### 2. 客户端代码（发送加法请求）

```c++
#include "ros/ros.h"
#include "my_package/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  ros::NodeHandle n;

  // 创建客户端，指定服务名称和类型
  ros::ServiceClient client = n.serviceClient<my_package::AddTwoInts>("add_two_ints");
  
  my_package::AddTwoInts srv;  // 服务消息对象
  srv.request.a = 10;          // 设置请求参数a
  srv.request.b = 20;          // 设置请求参数b

  // 发送请求并等待响应（阻塞调用）
  if (client.call(srv))
  {
    ROS_INFO("求和结果: %ld", (long int)srv.response.sum);  // 处理响应
  }
  else
  {
    ROS_ERROR("请求失败！");  // 请求失败处理
    return 1;
  }

  return 0;
}
```



## 四、Launch 文件的编写

### 1. 基本结构规范

Launch 文件采用 XML 格式，基本结构如下：

```xml
<launch>
  <!-- 节点、参数、组等内容 -->
</launch>
```

- 根标签必须是`<launch>`，所有内容需包含在其中
- XML 声明可选（`<?xml version="1.0"?>`），但建议添加以明确版本

### 2. 节点（node）定义规范

节点是 Launch 文件的核心元素，规范如下：

```xml
<node 
  pkg="package_name"        <!-- 必选，节点所在包名 -->
  type="executable_name"    <!-- 必选，可执行文件名 -->
  name="node_name"          <!-- 必选，节点运行时名称（覆盖代码中定义） -->
  output="screen"           <!-- 可选，日志输出到屏幕（默认文件） -->
  respawn="true"            <!-- 可选，节点退出后自动重启 -->
  required="true"           <!-- 可选，此节点退出则关闭整个launch -->
  ns="namespace"            <!-- 可选，设置节点命名空间 -->
  args="arg1 arg2"          <!-- 可选，传递命令行参数 -->
/>
```

- `name`应具有唯一性，避免重名导致冲突
- 建议添加`output="screen"`方便调试（生产环境可省略）
- 命名空间（`ns`）用于隔离不同功能模块的节点

For example:

```xml
<node 
      name="gazebo" 
      pkg="gazebo"  
      type="gazebo"   
      args="$(find r2_gazebo)/gazebo /r2_grap.world"    
      output="screen" 
      respawn="false"
/>

```



### 3. 参数（param/rosparam）设置规范

#### 单个参数（param）：

```xml
<param 
  name="param_name"         <!-- 必选，参数名 -->
  value="10"                <!-- 必选，参数值（支持数字、字符串、布尔等） -->
  type="int"                <!-- 可选，指定类型（int/string/bool/double） -->
  ns="namespace"            <!-- 可选，参数命名空间 -->
/>
```

#### 批量参数（rosparam）：

```xml
<!-- 优先：从YAML文件加载参数 -->
<rosparam command="load" file="$(find pkg)/config/params.yaml" ns="nav"/>
```

- 参数应按功能模块分组（通过`ns`或 YAML 文件）
- 建议将大量参数存入 YAML 文件，通过`rosparam`加载
- 避免硬编码敏感参数，优先使用`$(arg)`或`$(env)`动态获取

For example:

```xml
<rosparam command="load" file="$(find r2_gazebo)/config/controllers/r2_impedance_controller.yaml"  />   
```

该命令的作用是：将r2_impedance_controller.yaml文件加载到Parameter Server(参数服务器)中，设置控制参数。



### 4.Include:

`<include>`可以在当前launch文件中调用另一个launch文件，这样有利于代码的复用，减少开发工作量。下面是使用`<include>`的例子：

```xml
<include file ="$(find pr2_controller_manager)/controller_manager.launch"/>
```

### 5.env:

`<env>`可以用来设置节点的环境变量，可以在`<launch>、<include>、<node>`等。`<env>`的参数如下：

```xml
  <env 
    name="enviroment-variable-name" :设置的环境变量的名字
  	value="environment-variable-value" :环境变量的值
  />
```

### 6.remap：

`<remap>`可以将一个参数名映射为另一个名字，它的参数如下：

```xml
<remap  
  from="original-name"
  to="new-name"
/>
```

### 7.arg:

`<arg>`用来定义一个局部参数，该参数只能在一个launch文件中使用。`<arg>`有三种使用方法：

```xml
<arg name="foo"/> :声明一个参数foo，后面需要给它赋值
<arg name="foo" default="1"/> :声明一个参数foo，它有一个默认值，该值可以被修改。
<arg name="foo" value="bar"/> :声明一个常量foo，它的值不能被修改。
```

For example:

```xml
<launch>
  <! --declare arg to be passed in-->
  <arg name="hoge"  default="false"/>	
  <! --read value of arg-->
  <param name="param"  value="$(arg hoge)"/>
</launch>

```

## 五、Node文件的编写

### 1.基本结构规范

1. **文件组织**

   - 节点代码通常放在功能包（Package）的`src`目录下
   - 头文件放在`include/<package_name>/`目录
   - 配置文件（如`.yaml`）放在`config`目录
   - 启动文件（`.launch`）放在`launch`目录

2. **代码框架**一个标准的 ROS 节点代码应包含：

   - 初始化 ROS 节点

     ​	**将一个普通进程注册为 ROS 网络中的一个节点，使其能够加入 ROS 生态系统并参与通信**。

   - 创建句柄（`ros::NodeHandle`）

     ​    **创建节点句柄（NodeHandle）** 是节点与 ROS 系统交互的核心机制，其核心意义在于**提供一个统一的接口，让节点能够访问和管理 ROS 的各种资源**，包括话题、服务、参数服务器等。

   - 定义订阅者（Subscriber）、发布者（Publisher）

   - 声明服务（Service）或客户端（Client）（如需要）

   - 实现核心逻辑

   - 进入事件循环（`ros::spin()`或`ros::spinOnce()`）

     

     最简的Node结构

     ```c++
     #include "ros/ros.h"
     #include "std_msgs/String.h"  // 使用标准字符串消息
     
     int main(int argc, char **argv)
     {
         // 初始化节点，节点名称为"simple_node"
         ros::init(argc, argv, "simple_node");   
         // 创建节点句柄，用于管理节点资源
         ros::NodeHandle nh;
         // 创建发布者，发布"chatter"话题，消息类型为std_msgs::String，队列长度10
         ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
         // 设置循环频率为10Hz
         ros::Rate loop_rate(10);
         int count = 0;
         // 节点运行循环，直到ROS系统关闭
         while (ros::ok())
         {
             // 创建消息对象并赋值
             std_msgs::String msg;
             std::stringstream ss;
             ss << "hello world " << count;
             msg.data = ss.str();
             // 打印日志信息
             ROS_INFO("%s", msg.data.c_str());
             // 发布消息
             chatter_pub.publish(msg);
             // 处理回调函数（当前无回调，可省略）
             ros::spinOnce();
             // 按照循环频率休眠
             loop_rate.sleep();
             ++count;
         }
         
         return 0;
     }
     ```

### 2.命名规范

1. **节点名称**
   - 使用小写字母，单词间用下划线分隔（snake_case）
   - 避免使用数字开头，不包含特殊字符
   - 示例：`image_processor`、`odometry_publisher`
2. **话题与服务名称**
   - 同样使用 snake_case 命名法
   - 用斜杠`/`表示命名空间，如`/camera/image_raw`
   - 避免使用全局名称（除非必要），优先使用相对名称
3. **变量与函数**
   - C++ 变量 / 函数：遵循 C++ 命名规范，变量用 snake_case，类成员加前缀 m_
   - Python 变量 / 函数：统一使用 snake_case

### 3.通信规范

1. **消息类型选择**

   - 优先使用 ROS 标准消息类型（如`sensor_msgs/Image`、`geometry_msgs/Pose`）
   - 自定义消息放在`msg`目录，命名用 PascalCase（如`ObjectDetection.msg`）
   - 消息字段名用 snake_case

2. **发布 / 订阅规范**

   - 发布者（Publisher）应指定队列长度（queue_size）

   - 订阅回调函数应尽量轻量化，避免耗时操作

   - 示例：

     cpp运行

     ```cpp
     ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("output_image", 10);
     ros::Subscriber image_sub = nh.subscribe("input_image", 10, imageCallback;
     ```

3. **服务规范**

   - 服务名称应体现功能，如`/get_robot_pose`
   - 服务回调函数应快速响应，避免阻塞
   - 自定义服务放在`srv`目录，命名用 PascalCase（如`GetPose.srv`）

### 4.参数管理规范

1. **参数命名**

   - 使用小写字母，单词间用下划线分隔
   - 按功能模块组织，如`camera/frame_rate`、`controller/kp`

2. **参数获取**

   - 优先使用`ros::param::get()`或`nodehandle.getParam()`

   - 必须提供默认值，确保节点在缺少参数时能正常启动

   - 示例：

     cpp运行

     ```cpp
     int frame_rate;
     nh.param<int>("frame_rate", frame_rate, 30); // 第三个参数为默认值
     ```

3. **参数服务器**

   - 避免在节点中硬编码参数值
   - 复杂参数应放在`.yaml`配置文件中，通过 launch 文件加载

### 5.错误处理规范

1. **异常处理**
   - 对可能失败的操作（如参数获取、话题订阅）进行检查
   - 使用`ROS_ERROR()`、`ROS_WARN()`等宏输出错误信息
   - 关键错误发生时应调用`ros::shutdown()`退出节点
2. **日志输出**
   - 按级别使用：`ROS_DEBUG()`、`ROS_INFO()`、`ROS_WARN()`、`ROS_ERROR()`、`ROS_FATAL()`
   - 日志信息应清晰明了，包含必要的上下文
   - 避免在循环中使用大量`ROS_INFO()`，以免影响性能

### 6.CMakeLists 与 Package.xml 规范

1. **CMakeLists.txt**
   - 正确设置`cmake_minimum_required`版本
   - 声明依赖项（`find_package(catkin REQUIRED COMPONENTS ...)`）
   - 正确添加可执行文件和链接库
   - 安装规则（`install(TARGETS ...)`）
2. **Package.xml**
   - 填写正确的包名、版本、维护者信息
   - 声明所有依赖（`build_depend`、`exec_depend`等）
   - 确保格式符合 ROS 包管理规范

### 7.其他注意事项

1. **命名空间**
   - 使用节点私有命名空间（`ros::NodeHandle nh("~")`）存储节点专属参数
   - 通过 launch 文件为节点指定命名空间，提高代码复用性
2. **时间与定时器**
   - 使用`ros::Time`和`ros::Duration`处理时间
   - 定时任务优先使用`ros::Timer`，而非自定义循环



## 重点函数解释

`ros::ok()`

`ros::ok()` 返回一个布尔值（`true` 或 `false`），用于表示：

- **`true`**：节点处于正常运行状态，可以继续处理消息、发布数据、调用服务等。
- **`false`**：节点需要终止运行（如收到关闭信号），应退出主循环并清理资源。

在编写 ROS 节点时，几乎所有包含持续逻辑的主循环都会使用 `ros::ok()` 作为判断条件，这是保证节点可靠运行的重要实践。