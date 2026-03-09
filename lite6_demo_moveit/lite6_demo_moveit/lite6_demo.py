import rclpy
import csv
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import time


class Lite6IKNode(Node):

    def __init__(self):

        super().__init__("lite6_demo")

        # MoveIt
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[f"joint{i}" for i in range(1, 7)],
            base_link_name="link_base",
            end_effector_name="link_eef",
            group_name="lite6",
        )

        # publishers
        self.cartesian_pub = self.create_publisher(
            Point,
            "/posicion_deseada",
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            "/q_deseada",
            10
        )

        self.run()

    def run(self):

        path = "/home/ed/xarm_ws/src/xarm_perturbations/xarm_perturbations/waypoints.csv"

        try:

            with open(path, mode='r', encoding='utf-8-sig') as f:

                reader = csv.reader(f)

                for i, row in enumerate(reader):

                    if not row:
                        continue

                    raw = [float(v) for v in row]

                    pos = raw[:3]

                    quat = [1.0, 0.0, 0.0, 0.0]

                    self.get_logger().info(f"Testing point {i}: {pos}")

                    q_sol = self.moveit2.compute_ik(
                        position=pos,
                        quat_xyzw=quat
                    )

                    if q_sol is not None:

                        q_vals = q_sol.position

                        self.get_logger().info(
                            f"Waypoint {i} | Pos: {pos} | "
                            f"q_des: {[round(q,3) for q in q_vals]}"
                        )

                        # publicar posicion cartesiana
                        p = Point()
                        p.x, p.y, p.z = pos
                        self.cartesian_pub.publish(p)
                        self.get_logger().info(
                        f"Publicado en /posicion_deseada -> "
                        f"[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"
                    )

                        # publicar joints
                        j = JointState()
                        j.header.stamp = self.get_clock().now().to_msg()
                        j.name = [
                            "joint1","joint2","joint3",
                            "joint4","joint5","joint6"
                        ]
                        j.position = list(q_vals)

                        self.joint_pub.publish(j)
                        self.get_logger().info(
                        f"Publicado en /q_deseada -> "
                        f"{[round(q,3) for q in j.position]}"
                        )

                    else:

                        self.get_logger().warn(
                            f"IK falló para waypoint {i}: {pos}"
                        )

                    self.get_logger().info(
                        f"Publicado waypoint {i} en t={time.time():.2f}"
                    )

                    time.sleep(10)

        except FileNotFoundError:

            self.get_logger().error("Archivo CSV no encontrado")


def main():

    rclpy.init()

    node = Lite6IKNode()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()