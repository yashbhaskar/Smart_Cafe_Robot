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

## 📹 Working Videos

https://drive.google.com/drive/folders/1HVsILKfBWDLQr8Mfs-OQ0qOrstMorvIb

## 📌 Handling Different Scenarios
| Scenario | Robot Behavior |
|----------|---------------|
| Order received | Moves from home → kitchen → table → home |
| No confirmation at kitchen | Waits for 10 second timeout, then returns home |
| No confirmation at table | Waits for 10 second timeout, then move kitchen before returning home |
| Task canceled | Returns to kitchen → home |
| Multiple orders (any order cancel) | Delivers to all tables move kitchen before returning home |
| Multiple orders (all confirm order) | Delivers to all tables and returning home |


# Decision Tree for Navigation

```mermaid
graph TD;
    A[Start] -->|Go to| B(Kitchen)
    B -->|Confirm| C(Table 1)
    B -->|Cancel| D(Home)
    C --> E{Next Table?}
    E -->|Table 2| F(Table 2)
    F -->|Table 3| G(Table 3)
    G --> H(Back to Home)
    E -->|No| H


## 📂 Project Structure
```
robot_description/
│── launch/                        # Launch files for ROS 2
│   ├── gazebo.launch.py
│   ├── state_publisher.launch.py
│── models/
│   ├── meshes
│   ├── urdf
│── scripts/                        # Python scripts for control and gui
│   ├── robot_controller.py
│── worlds/
│   ├── cafe.sdf
│── CMakeLists.txt                  # CMake build configuration
│── package.xml
```

## 📡 RQT Graph Visualization
Below is an RQT graph of the ROS 2 nodes and topics used in this package:

![Screenshot from 2025-03-02 02-52-34](https://github.com/user-attachments/assets/a9a93ff0-0ddd-466b-9f58-6d6e24d8106f)


## 🤝 Contributing

Feel free to fork this repository, create a pull request, or open an issue if you have suggestions or find bugs.

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com
📌 GitHub: https://github.com/yashbhaskar
