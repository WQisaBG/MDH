# MDH

#### **1. 确定可执行文件路径**

- 在 ROS2 项目中，可执行文件通常位于 `install` 或 `build` 目录下。

- 使用以下命令查找可执行文件路径：

  ```bash
  
  
  find /home/abc/MDH/ros_ws -name motor_control_node
  ```

- 假设输出为 `/home/abc/MDH/ros_ws/install/motor_control_v2/lib/motor_control_v2/motor_control_node`。

#### **2. 加载可执行文件到 GDB**

- 使用 GDB 的 `file`命令加载可执行文件：

  ```bash
  
  
  gdb /home/abc/MDH/ros_ws/install/motor_control_v2/lib/motor_control_v2/motor_control_node
  ```

#### **3. 设置环境变量**

- ROS2 程序需要正确的环境变量（如 `ROS_DOMAIN_ID` 和 `AMENT_PREFIX_PATH`）。

- 启动 GDB 前，确保已 source 工作空间：

  ```bash
  
  
  source /home/abc/MDH/ros_ws/install/setup.bash
  ```

- 或者在 GDB 中设置环境变量：

  ```bash
  (gdb) set env ROS_DOMAIN_ID=0
  (gdb) set env AMENT_PREFIX_PATH=/home/abc/MDH/ros_ws/install
  ```

#### **4. 运行程序**

- 使用 GDB 的run命令运行程序，并传递参数：

  ```bash
  (gdb) run config.json
  ```

------

### **完整调试流程**

1. **打开 GDB 并加载可执行文件**：

   ```bash
   gdb /home/abc/MDH/ros_ws/install/motor_control_v2/lib/motor_control_v2/motor_control_node
   ```

2. **设置环境变量**（如果需要）：

   ```bash
   (gdb) set env ROS_DOMAIN_ID=0
   (gdb) set env AMENT_PREFIX_PATH=/home/abc/MDH/ros_ws/install
   ```

3. **运行程序并传递参数**：

   ```bash
   (gdb) run config.json
   ```

4. **捕获崩溃堆栈信息**：

   - 如果程序崩溃，使用以下命令查看堆栈信息：

     ```bash
     (gdb) bt
     ```

