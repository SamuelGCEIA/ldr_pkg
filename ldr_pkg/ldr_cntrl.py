#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
import xacro
import os
import xml.etree.ElementTree as ET
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LdrCntrl(Node):
    def __init__(self):
        # Nombre del nodo
        super().__init__("ldr_cntrl")
        self.threshold_distance = 0.35 # Umbral de distancia
        self.subscription = self.create_subscription(LaserScan, "/scan", self.ldr_callback, 10) # Suscripción al tópico de Láser
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10) # Publicación al tópico de Velocidad
        # Inicializar temporizador
        self.timer=None
        self.twist = Twist() # Objeto de tipo Twist

    def ldr_callback(self, msg):
        min_range = min(msg.ranges) # Obtiene la distancia
        self.get_logger().info(f"Min range: {min_range:.2f} m") # Imprime la distancia

        twist = Twist() # Objeto de tipo Twist
        if min_range <= self.threshold_distance: # Si la distancia es menor o igual al umbral
            self.get_logger().info("Obstacle detected") # Imprime si detecta un obstáculo
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.0
            self.publisher.publish(self.twist) # Publica el mensaje
            # Temporizador para esperar 3 segundos
            self.timer = self.create_timer(3.0, self.chek_position_after_turn)

    def chek_position_after_turn(self, msg):
        min_range = min(msg.ranges) # Obtiene la distancia
        self.get_logger().info(f"Min range after turn: {min_range:.2f} m")
        if min_range <= self.threshold_distance:
            self.get_logger().info("Obstacle detected after turn")
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            
        else:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
        
        self.timer.cancel() # Cancela el temporizador

def main(args=None):
    rclpy.init(args=args)
    node = LdrCntrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()