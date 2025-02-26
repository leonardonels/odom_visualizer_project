import rclpy
from rclpy.node import Node
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            Odometry,
            'debug/odometry',
            self.odom_callback,
            qos_profile)

        self.circuit_data = pd.read_csv('vallelunga1_circuit.csv')
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.circuit_data['x'], self.circuit_data['y'], label='circuit')
        self.point_plot, = self.ax.plot([], [], 'ro', markersize=8, label='closest point')
        self.external_plot, = self.ax.plot([], [], 'bo', markersize=6, label='car')
        self.vector_plot = self.ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1, color='g', label='yaw')
        self.ax.legend()
        plt.ion()
        plt.show()
    
    def odom_callback(self, msg):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        
        orient_x = msg.pose.pose.orientation.z
        orient_y = msg.pose.pose.orientation.w
        
        self.point_plot.set_data(pos_x, pos_y)
        
        self.external_plot.set_data(orient_x, orient_y)
        
        vector_length = 1.0
        angle = pos_z
        vec_x = vector_length * np.cos(angle)
        vec_y = vector_length * np.sin(angle)
        self.vector_plot.set_offsets([[orient_x, orient_y]])
        self.vector_plot.set_UVC(vec_x, vec_y)
        
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    visualizer_node = VisualizerNode()
    rclpy.spin(visualizer_node)
    visualizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
