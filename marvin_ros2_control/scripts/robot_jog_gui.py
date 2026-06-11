#!/usr/bin/env python3

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from marvin_msgs.msg import JointcmdArm, JointcmdBody, JointcmdHead


class JointPanel:
    def __init__(self, parent, title, names, min_value, max_value, step_getter, command_callback):
        self.names = list(names)
        self.command_callback = command_callback
        self.step_getter = step_getter
        self.values = [0.0 for _ in self.names]
        self.scales = []
        self.labels = []

        frame = ttk.LabelFrame(parent, text=title, padding=8)
        frame.pack(fill=tk.X, padx=8, pady=6)

        for index, name in enumerate(self.names):
            row = ttk.Frame(frame)
            row.pack(fill=tk.X, pady=2)

            ttk.Label(row, text=name, width=22, anchor=tk.W).pack(side=tk.LEFT)
            minus_button = ttk.Button(row, text="-", width=3, command=lambda i=index: self.jog(i, -1.0))
            minus_button.pack(side=tk.LEFT, padx=(0, 4))

            value = tk.DoubleVar(value=0.0)
            scale = ttk.Scale(row, from_=min_value, to=max_value, orient=tk.HORIZONTAL, variable=value)
            scale.state(["disabled"])
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.scales.append(value)

            plus_button = ttk.Button(row, text="+", width=3, command=lambda i=index: self.jog(i, 1.0))
            plus_button.pack(side=tk.LEFT, padx=(4, 8))

            label = ttk.Label(row, text="0.0000", width=10, anchor=tk.E)
            label.pack(side=tk.LEFT)
            self.labels.append(label)

    def update_values(self, values):
        for index, value in enumerate(values[:len(self.names)]):
            self.values[index] = float(value)
            self.scales[index].set(float(value))
            self.labels[index].configure(text=f"{float(value):.4f}")

    def jog(self, index, direction):
        values = list(self.values)
        values[index] += direction * self.step_getter()
        self.command_callback(values)


class RobotJogGui(Node):
    def __init__(self):
        super().__init__("marvin_robot_jog_gui")

        self.declare_parameter("left_joints", [f"Joint{i}_L" for i in range(1, 8)])
        self.declare_parameter("right_joints", [f"Joint{i}_R" for i in range(1, 8)])
        self.declare_parameter("head_joints", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("body_joints", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("lift_joints", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("jog_step", 0.01)
        self.declare_parameter("lift_jog_step", 0.005)

        self.left_names = list(self.get_parameter("left_joints").value)
        self.right_names = list(self.get_parameter("right_joints").value)
        self.head_names = self.get_string_array_parameter("head_joints")
        self.lift_names = self.get_string_array_parameter("lift_joints")
        self.body_names = self.get_string_array_parameter("body_joints")
        self.body_control_names = self.lift_names + self.body_names

        self.left_cmd = [0.0] * 7
        self.right_cmd = [0.0] * 7
        self.head_cmd = [0.0] * 3
        self.body_cmd = [0.0] * 6
        self.lock = threading.Lock()
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.left_pub = self.create_publisher(JointcmdArm, "control/joint_cmd_A", control_qos)
        self.right_pub = self.create_publisher(JointcmdArm, "control/joint_cmd_B", control_qos)
        self.body_pub = self.create_publisher(JointcmdBody, "control/joint_cmd_body", control_qos)
        self.head_pub = self.create_publisher(JointcmdHead, "control/joint_cmd_head", control_qos)

        self.create_subscription(JointcmdArm, "info/cmd_pos_A", self.left_cmd_callback, qos_profile_sensor_data)
        self.create_subscription(JointcmdArm, "info/cmd_pos_B", self.right_cmd_callback, qos_profile_sensor_data)
        self.create_subscription(JointcmdBody, "info/cmd_pos_body", self.body_cmd_callback, qos_profile_sensor_data)
        self.create_subscription(JointcmdHead, "info/cmd_pos_head", self.head_cmd_callback, qos_profile_sensor_data)

        self.root = tk.Tk()
        self.root.title("Marvin Robot Jog")
        self.root.geometry("980x760")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="Jog step rad").pack(side=tk.LEFT)
        self.step_var = tk.DoubleVar(value=float(self.get_parameter("jog_step").value))
        ttk.Spinbox(controls, from_=0.0001, to=0.5, increment=0.001, textvariable=self.step_var, width=8).pack(side=tk.LEFT, padx=(4, 16))
        ttk.Label(controls, text="Lift step m").pack(side=tk.LEFT)
        self.lift_step_var = tk.DoubleVar(value=float(self.get_parameter("lift_jog_step").value))
        ttk.Spinbox(controls, from_=0.0001, to=0.2, increment=0.001, textvariable=self.lift_step_var, width=8).pack(side=tk.LEFT, padx=(4, 16))

        canvas = tk.Canvas(self.root, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.root, orient=tk.VERTICAL, command=canvas.yview)
        self.content = ttk.Frame(canvas)
        self.content.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.content, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.left_panel = JointPanel(self.content, "Left Arm", self.left_names[:7], -math.pi, math.pi, self.get_joint_step, self.publish_left)
        self.right_panel = JointPanel(self.content, "Right Arm", self.right_names[:7], -math.pi, math.pi, self.get_joint_step, self.publish_right)
        self.body_panel = JointPanel(self.content, "Body Control", self.body_control_names[:6], -math.pi, math.pi, self.get_body_step_for_display, self.publish_body)
        self.head_panel = JointPanel(self.content, "Head", self.head_names, -math.pi, math.pi, self.get_joint_step, self.publish_head)

        self.root.after(50, self.refresh_gui)

    def get_joint_step(self):
        return float(self.step_var.get())

    def get_string_array_parameter(self, name):
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if value is None:
            return []
        return list(value)

    def get_lift_step(self):
        return float(self.lift_step_var.get())

    def get_body_step_for_display(self):
        return float(self.step_var.get())

    def left_cmd_callback(self, msg):
        with self.lock:
            self.left_cmd = list(msg.positions)

    def right_cmd_callback(self, msg):
        with self.lock:
            self.right_cmd = list(msg.positions)

    def body_cmd_callback(self, msg):
        with self.lock:
            self.body_cmd = list(msg.positions)

    def head_cmd_callback(self, msg):
        with self.lock:
            self.head_cmd = list(msg.positions)

    def refresh_gui(self):
        with self.lock:
            left = list(self.left_cmd)
            right = list(self.right_cmd)
            body = list(self.body_cmd)
            head = list(self.head_cmd)

        self.left_panel.update_values(left)
        self.right_panel.update_values(right)
        self.body_panel.update_values(body)
        self.head_panel.update_values(head)
        self.root.after(50, self.refresh_gui)

    def publish_left(self, values):
        self.left_cmd = self.pad(values, 7)
        msg = JointcmdArm()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.positions = self.left_cmd
        self.left_pub.publish(msg)

    def publish_right(self, values):
        self.right_cmd = self.pad(values, 7)
        msg = JointcmdArm()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.positions = self.right_cmd
        self.right_pub.publish(msg)

    def publish_body(self, values):
        adjusted = list(values)
        if adjusted and self.lift_names:
            lift_delta = adjusted[0] - self.body_cmd[0]
            if lift_delta != 0.0:
                adjusted[0] = self.body_cmd[0] + math.copysign(self.get_lift_step(), lift_delta)
        self.body_cmd = self.pad(adjusted, 6)
        msg = JointcmdBody()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.positions = self.body_cmd
        self.body_pub.publish(msg)

    def publish_head(self, values):
        self.head_cmd = self.pad(values, 3)
        msg = JointcmdHead()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.positions = self.head_cmd
        self.head_pub.publish(msg)

    def pad(self, values, size):
        padded = list(values[:size])
        while len(padded) < size:
            padded.append(0.0)
        return padded

    def close(self):
        self.root.quit()


def main():
    rclpy.init()
    node = RobotJogGui()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
