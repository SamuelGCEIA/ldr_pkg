#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
import xml.etree.ElementTree as ET
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LdrCntrl(Node):
    def _init_(self):
        # Nombre del nodo
        super()._init_("ldr_cntrl")
        self.threshold_distance = 0.35 # Umbral de distancia
        self.subscription = self.create_subscription(LaserScan, "/scan", self.ldr_callback, 10) # Suscripción al tópico de Láser
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10) # Publicación al tópico de Velocidad
        # Inicializar temporizador
        self.timer=None
        self.twist = Twist() # Objeto de tipo Twist
        self.bandera = True
    
    def ldr_callback(self, msg):
        min_range = min(msg.ranges) # Obtiene la distancia
        self.get_logger().info(f"Min range: {min_range:.2f} m") # Imprime la distancia
        twist = Twist() # Objeto de tipo Twist
        if min_range <= self.threshold_distance and self.bandera == True: # Si la distancia es menor o igual al umbral
            self.bandera = False
            self.get_logger().info("Obstacle detected") # Imprime si detecta un obstáculo
            self.twist.linear.x = 0.0
            self.twist.angular.z = -1.0
            self.publisher.publish(self.twist) # Publica el mensaje
            # Temporizador para esperar 3 segundos
            self.timer = self.create_timer(1.0, self.chek_position_after_turn)
        elif min_range >= self.threshold_distance:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist) # Publica el mensaje           
    



    def chek_position_after_turn(self, msg):
        min_range = min(msg.ranges) # Obtiene la distancia
        self.get_logger().info(f"Min range after turn: {min_range:.2f} m")
        if min_range <= self.threshold_distance:
            self.get_logger().info("Obstacle detected after turn")
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            self.bandera = True
            self.timer.cancel()
        

def main(args=None):
    rclpy.init(args=args)
    node = LdrCntrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "_main_":
    main()