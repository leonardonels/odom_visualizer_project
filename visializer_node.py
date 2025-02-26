import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.subscription = self.create_subscription(
            Odometry,
            'debug/odometry',
            self.odom_callback,
            qos_profile)

        # Carica il circuito dal file CSV
        self.circuit_data = pd.read_csv('vallelunga1_circuit.csv')
        self.fig, self.ax = plt.subplots()
        self.ax.plot(self.circuit_data['x'], self.circuit_data['y'], label='Circuito')
        self.point_plot, = self.ax.plot([], [], 'ro', markersize=8, label='Posizione attuale')
        self.external_plot, = self.ax.plot([], [], 'bo', markersize=6, label='Punto esterno')
        self.ax.legend()
        plt.ion()
        plt.show()
    
    def odom_callback(self, msg):
        # Estrai i dati dalla posizione e orientamento
        nn_pos_x = msg.pose.pose.position.x
        nn_pos_y = msg.pose.pose.position.y
        yaw = msg.pose.pose.position.z  # Usato per l'angolo
        
        pos_x = msg.pose.pose.orientation.z  # Punto esterno x
        pos_y = msg.pose.pose.orientation.w  # Punto esterno y
        
        # Aggiorna il punto attuale sul circuito
        self.point_plot.set_data(nn_pos_x, nn_pos_y)
        
        # Aggiorna il punto esterno
        self.external_plot.set_data(pos_x, pos_y)
        
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    visualizer_node = VisualizerNode()
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            visualizer_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
