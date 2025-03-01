# 🚀 CaféBot: Smart Cafe Robot Using ROS2

CaféBot is an intelligent butler robot designed to streamline food delivery operations in busy cafés. It automates order pickup and delivery, reducing employee workload and improving efficiency. The robot follows a structured workflow—starting from the home position, picking up food from the kitchen, and delivering it to customer tables. It handles multiple orders, cancellations, and confirmation-based interactions, ensuring a smooth dining experience.
---

## 🚀 Features  
✅ **Autonomous navigation from home → kitchen → tables → home**
✅ **Handles multiple simultaneous orders efficiently**
✅ **Timeout mechanism if confirmation is not received**
✅ **Smart return logic (e.g., returning to the kitchen after cancellations)**
✅ **Adaptive to busy café environments** 

---

## 🛠️ Installation  

### 1️⃣ **Clone the Repository**  
```bash
cd ~/ros_ws/src
git clone https://github.com/yashbhaskar/Smart_Cafe_Robot.git
cd ~/ros_ws
```

### 2️⃣ **Build the Package** 
```bash
colcon build --packages-select robot_description
source install/setup.bash
```

## 🎮 Usage

### 1️⃣ Launch Gazebo and State_Publisher:
```bash
ros2 launch robot_description gazebo.launch.py
ros2 launch robot_description state_publisher.launch.py
```
![Screenshot from 2025-03-02 02-38-27](https://github.com/user-attachments/assets/84597889-09bd-4ac2-ad3b-f2dba351bae5)
![Screenshot from 2025-03-02 02-39-27](https://github.com/user-attachments/assets/8ef47259-9354-41f7-88e6-0e2f0b051700)

### 2️⃣ Run the Robot Controller Command Node:
```bash
ros2 run robot_description robot_controller.py
```

### 3️⃣ Give Table inputs on gui and Start:
```bash
table1,table2,table3
```
![Screenshot from 2025-03-02 02-42-27](https://github.com/user-attachments/assets/62d263ae-bd8f-44b9-8d6f-c698fe49207e)

### 4️⃣ Give input Confirm/Cancel:
Once the node starts, it will listen for voice commands such as:
``"If Confirm"`` – Move towards tables for delivery
``"If Cancel"`` – Move towards Home
![Screenshot from 2025-03-02 02-43-51](https://github.com/user-attachments/assets/b6c0c0b1-d167-4890-bde1-5d54251bb14e)

## 📹 Demo Video

https://github.com/user-attachments/assets/ffe5805c-e38b-40e1-bdbb-48c0156fc791

## 📡 ROS 2 Topics & Services Used

| Topic Name   | Message Type      | Description             |
|--------------|-------------------|--------------------------|
| `/cmd_vel` | `geometry_msgs/Twist`| Publishes velocity commands to the robot|
|`/voice_command` | `std_msgs/String`| Publishes recognized voice commands |


## 📂 Project Structure
```
ros2_bot_description/
│── launch/                        # Launch files for ROS 2
│   ├── gazebo.launch.py
│   ├── state_publisher.launch.py
│   ├── slave.launch.py
│── models/
│   ├── meshes
│   ├── urdf
│── scripts/                        # Python scripts for voice recognition
│   ├── robot_controller.py
│   ├── voice_command.py
│── worlds/
│   ├── new_world.sdf
│── CMakeLists.txt                  # CMake build configuration
│── package.xml
│── README.md
```

## 📡 RQT Graph Visualization
Below is an RQT graph of the ROS 2 nodes and topics used in this package:

![Screenshot from 2025-02-09 16-07-54](https://github.com/user-attachments/assets/36fdc976-b912-4bae-b3a4-cb0c9236f477)

This shows how voice commands are processed and sent to the robot.

## 🤝 Contributing

Feel free to fork this repository, create a pull request, or open an issue if you have suggestions or find bugs.

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com
📌 GitHub: https://github.com/yashbhaskar
