import rclpy
import csv
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point

class MoveItIKProvider(Node):
    def __init__(self):
        super().__init__("lite6_node")
        
        # Configuración de PyMoveIt2
        self.moveit2 = MoveIt2(
            node=Node,
            joint_names=[f"joint{i}" for i in range(1, 7)],
            base_link_name="link_base",
            end_effector_name="link_eef",
            group_name="lite6",
        )

        self.q_des_pub = self.create_publisher(JointState, "/q_deseada", 10)
        self.cartesian_pub = self.create_publisher(Point, "/posicion_actual_deseada", 10)
        
        self.waypoints = self.load_csv("/home/ed/xam_ws/src/lite6_demo_moveit/lite6_demo_moveit/trayectoria_lite6.csv")
        self.current_idx = 0
        
        # Timer para enviar puntos a la misma frecuencia que el control (100Hz)
        self.timer = self.create_timer(10, self.publish_next_ik)

    def load_csv(self, path):
        try:
            with open(path, mode='r', encoding='utf-8-sig') as f:
                return [list(map(float, row)) for row in csv.reader(f) if row]
        except FileNotFoundError:
            self.get_logger().error(f"No se encontró el archivo: {path}")
            return []
           
            
    def publish_next_ik(self):
        if self.current_idx >= len(self.waypoints):
            return

        pos = self.waypoints[self.current_idx][:3]
        quat = [1.0, 0.0, 0.0, 0.0] # Orientación fija

        # 1. PASAR POSICIÓN A MOVEIT PARA CALCULAR Q
        # Usamos el modelo cinemático interno para obtener la solución
        q_solucion = self.moveit2.compute_inverse_kinematics(position=pos, quat_xyzw=quat)

        if q_solucion is not None:
            # A. PUBLICAR JOINTS (Para el CTC)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.position = [float(q) for q in q_solucion]
            self.q_des_pub.publish(joint_msg)

            # B. PUBLICAR COORDENADA (Para monitoreo o RMSE)
            point_msg = Point()
            point_msg.x = pos[0]
            point_msg.y = pos[1]
            point_msg.z = pos[2]
            self.cartesian_pub.publish(point_msg)
            self.get_logger().info(f"Waypoint {self.current_idx} | "f"Pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] | " f"Joints: {[round(q,3) for q in q_solucion]}"
)

            self.current_idx += 1
        else:
            self.get_logger().warn(f"IK falló para el punto {self.current_idx}: {pos}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItIKProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
