#!/usr/bin/env python3

import rclpy
import math
import time
import sys
import select
import threading
import tkinter as tk
from tkinter import messagebox
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WAYPOINTS = {
    "home": (0.0, 0.0),
    "kitchen": (-0.105, 6.085),
    "table1": (0.7907, -5.8238),
    "table2": (4.503, -8.776),
    "table3": (5.205, -3.553)
}

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.task_queue = []
        self.reached_goal = False
        self.awaiting_confirmation = False
        self.canceled_orders = False  # Track canceled orders
        self.kitchen_after_cancel = False  # Track if kitchen visit is needed after cancellations
        self.last_log_time = time.time()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
        if self.task_queue and not self.awaiting_confirmation:
            self.execute_task()

    def quaternion_to_euler(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)
        return 0.0, 0.0, yaw
    
    def move_to_goal(self, goal_x, goal_y):
        cmd = Twist()
        distance = math.sqrt((goal_x - self.current_x) ** 2 + (goal_y - self.current_y) ** 2)
        desired_yaw = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        if distance > 0.02:
            if abs(yaw_error) > 0.02:
                cmd.angular.z = 5.0 * yaw_error
            else:
                cmd.linear.x = min(3.5, distance)
        else:
            cmd.linear.x, cmd.angular.z = 0.0, 0.0
            self.reached_goal = True
        
        self.publisher_.publish(cmd)

    def execute_task(self):
        if not self.task_queue:
            return
        
        task = self.task_queue[0]
        
        if self.reached_goal:
            time.sleep(0.5)
            
            if task == "kitchen":
                if self.kitchen_after_cancel:
                    self.log_info("Returning to kitchen after cancellations.")
                    self.kitchen_after_cancel = False
                else:
                    confirm = gui.ask_confirm(f"At {task}. Confirm order?")
                    if not confirm:
                        self.task_queue = ["home"]
                        self.canceled_orders = False
                        return
            elif "table" in task:
                confirm = gui.ask_confirm(f"At {task}. Confirm order?")
                if not confirm:
                    self.canceled_orders = True
            
            self.task_queue.pop(0)
            self.reached_goal = False
            if not self.task_queue and self.canceled_orders:
                self.task_queue.append("kitchen")
                self.kitchen_after_cancel = True
                self.canceled_orders = False
            if not self.task_queue:
                self.task_queue.append("home")
        else:
            self.log_info(f"Moving to {task}...")
            self.move_to_goal(*WAYPOINTS[task])
    
    def process_orders(self, orders):
        self.task_queue = ["kitchen"] + orders
        self.kitchen_after_cancel = False
        self.log_info(f"Processing orders: {orders}")
    
    def log_info(self, message):
        current_time = time.time()
        if current_time - self.last_log_time >= 10:
            self.get_logger().info(message)
            gui.update_status(message)
            self.last_log_time = current_time


class RobotGUI:
    def __init__(self, root, robot_controller):
        self.root = root
        self.robot_controller = robot_controller
        self.root.title("Robot Controller")

        self.label = tk.Label(root, text="Enter table orders:")
        self.label.pack()

        self.entry = tk.Entry(root)
        self.entry.pack()

        self.start_button = tk.Button(root, text="Start", command=self.start_robot)
        self.start_button.pack()

        self.cancel_button = tk.Button(root, text="Cancel", command=self.cancel_anytime)
        self.cancel_button.pack()

        self.status_label = tk.Label(root, text="Status: Idle")
        self.status_label.pack()

    def start_robot(self):
        orders = self.entry.get().split(',')
        orders = [o.strip() for o in orders if o.strip() in WAYPOINTS]
        if orders:
            self.robot_controller.process_orders(orders)
        else:
            messagebox.showwarning("Input Error", "Enter valid table names (e.g., table1, table2)")

    def cancel_anytime(self):
        """Cancel the mission anytime before reaching a table, redirecting to  home."""
        confirmed = self.ask_confirm("Cancel Order? Robot will return to home.")
        if confirmed:
            self.robot_controller.task_queue = ["home"]  
            self.update_status("Mission canceled! Returning to home.")

    def cancel_mission(self):
        """Handles canceling the order at the table (same behavior as cancel_anytime)."""
        self.cancel_anytime()

    def ask_confirm(self, message):
        """Popup confirmation dialog with 10s auto-cancel"""
        confirm_window = tk.Toplevel(self.root)
        confirm_window.title("Order Confirmation")
        tk.Label(confirm_window, text=message).pack()

        response_var = tk.BooleanVar(value=False)

        def on_confirm():
            response_var.set(True)
            confirm_window.destroy()

        def on_cancel():
            response_var.set(False)
            confirm_window.destroy()

        confirm_button = tk.Button(confirm_window, text="Confirm", command=on_confirm)
        cancel_button = tk.Button(confirm_window, text="Cancel", command=on_cancel)

        confirm_button.pack()
        cancel_button.pack()
        
        confirm_window.update_idletasks()
        confirm_window.grab_set()

        def auto_cancel():
            if confirm_window.winfo_exists():
                response_var.set(False)
                confirm_window.destroy()

        confirm_window.after(10000, auto_cancel)

        self.root.wait_window(confirm_window)  # Wait until the popup closes

        return response_var.get()
        

    def update_status(self, message):
        self.status_label.config(text=f"Status: {message}")




def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    global gui
    root = tk.Tk()
    gui = RobotGUI(root, robot_controller)
    
    thread = threading.Thread(target=rclpy.spin, args=(robot_controller,), daemon=True)
    thread.start()
    
    root.mainloop()
    
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

