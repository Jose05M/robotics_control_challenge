#!/usr/bin/env python3

import math
import numpy as np
import csv
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from tf2_ros import Buffer, TransformListener
from pynput import keyboard
from enum import Enum
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog

# =============================================================
# LITE6 DYNAMICS
# =============================================================

DH = np.array([
    [0.0,       0.2433, 0.0,   np.pi/2],
    [-np.pi/2,  0.0,    0.200, 0.0    ],
    [np.pi/2,   0.0,    0.0,   np.pi/2],
    [0.0,       0.2276, 0.0,  -np.pi/2],
    [0.0,       0.0,    0.0,   np.pi/2],
    [0.0,       0.0615, 0.0,   0.0    ],
])

MASSES = np.array([1.411, 1.34, 0.953, 1.284, 0.804, 0.13])

def inertia_matrix(q):
    base = 0.8 + 2.5 * (MASSES / MASSES.max())
    diag = base + 0.25 * np.cos(q)
    diag = np.maximum(diag, 0.2)
    M = np.diag(diag)
    M += 0.05*np.eye(6)
    return M

def coriolis_torque(q, qd):
    return 0.12*qd + 0.03*np.sin(q) * np.roll(qd, 1)

def gravity_torque(q):
    g = 9.81
    return np.ones(6) * g * 0.3


class RobotState(Enum):
    RUNNING = 1
    PAUSED = 2


class CircleServoXArmLite6(Node):

    def __init__(self):
        super().__init__("circle_servo_xarm_lite6")

        # -----------------------------------------------------
        # SUBSCRIPTIONS
        # -----------------------------------------------------

        self.create_subscription(JointState, "/q_deseada", self.q_des_callback, 10)
        self.create_subscription(Point, "/posicion_deseada", self.cartesian_des_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)

        # -----------------------------------------------------
        # STATE VARIABLES
        # -----------------------------------------------------
        # trayectoria CTC
        self.q_start = None
        self.q_goal = None
        self.traj_start_time = None
        self.traj_duration = 3.0

        self.q = None
        self.qd = None
        self.q_des_actual = None
        self.target_pos_cartesian = None

        self.prev_error_cart = np.zeros(3)

        self.dt = 0.1
        self.print_counter = 0
        self.goal_tolerance = 0.005  # 5 mm
        self.goal_reached = False

        # -----------------------------------------------------
        # PUBLISHERS
        # -----------------------------------------------------

        self.servo_pub = self.create_publisher(
            TwistStamped,
            "/servo_server/delta_twist_cmds",
            10
        )

        self.joint_pub = self.create_publisher(
            JointJog,
            "/servo_server/delta_joint_cmds",
            10
        )

        # -----------------------------------------------------
        # CONTROL CONFIG
        # -----------------------------------------------------

        self.robot_state = RobotState.RUNNING
        self.control_mode = "CTC"

        self.kp_cart = np.array([3.5, 3.5, 3.5])
        self.kd_cart = np.array([0.5, 0.5, 0.1])

        self.max_speed = 0.22
        self.epsilon = 0.0005

        self.Kp_ctc = np.diag([60,40,30,20,15,11])
        self.Kd_ctc = np.diag([10,6,4,2,1,0.2])
        

        self.tau_limit = 50.0

        # -----------------------------------------------------
        # TF
        # -----------------------------------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------------------------------
        # METRICS
        # -----------------------------------------------------

        self.rmse_sum = np.zeros(3)
        self.rmse_count = 0

        self.start_time = self.get_clock().now()
        self.last_info_time = self.get_clock().now()

        # -----------------------------------------------------
        # CSV LOGGING
        # -----------------------------------------------------

        self.csv_file = open("tracking_data_lite6.csv","w",newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "time",

            "x_d","y_d","z_d",
            "x","y","z",

            "vx_cmd","vy_cmd","vz_cmd",

            "q1","q2","q3","q4","q5","q6",
            "qd1","qd2","qd3","qd4","qd5","qd6",

            "qref1","qref2","qref3","qref4","qref5","qref6",

            "cmd1","cmd2","cmd3","cmd4","cmd5","cmd6",

            "mode",
            "perturbation"
            ])

        # -----------------------------------------------------
        # KEYBOARD CONTROL
        # -----------------------------------------------------

        self._start_keyboard()

        # -----------------------------------------------------
        # MAIN LOOP
        # -----------------------------------------------------

        self.timer = self.create_timer(self.dt, self._loop)

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------

    def joint_callback(self,msg):

        if len(msg.position) >= 6:
            self.q = np.array(msg.position[:6])

        if len(msg.velocity) >= 6:
            self.qd = np.array(msg.velocity[:6])
        else:
            self.qd = np.zeros(6)

    def q_des_callback(self,msg):
        if len(msg.position) >= 6:
            new_goal = np.array(msg.position[:6])
            # iniciar nueva trayectoria
            if self.q is not None:
                self.q_start = self.q.copy()
                self.q_goal = new_goal.copy()
                self.traj_start_time = self.get_clock().now()
            self.q_des_actual = new_goal

    def cartesian_des_callback(self,msg):

        self.target_pos_cartesian = np.array([
            msg.x,msg.y,msg.z
        ])

        self.goal_reached = False
        self.prev_error_cart = np.zeros(3)


    def _generate_ctc_trajectory(self):
        if self.q_start is None or self.q_goal is None:
            return None, None, None
        now = self.get_clock().now()
        t = (now - self.traj_start_time).nanoseconds / 1e9
        T = self.traj_duration
        if t >= T:
            return self.q_goal, np.zeros(6), np.zeros(6)
        s = t / T
        q0 = self.q_start
        q1 = self.q_goal
        q_d = q0 + (3*s**2 - 2*s**3) * (q1 - q0)
        qd_d = (6*s*(1-s)/T) * (q1 - q0)
        qdd_d = (6*(1-2*s)/(T**2)) * (q1 - q0)
        return q_d, qd_d, qdd_d

    # ---------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------

    def _loop(self):

        if (
            self.robot_state == RobotState.PAUSED
            or self.q is None
            or self.qd is None
        ):
            return

        current_cartesian = self._read_pose()

        if current_cartesian is None:
            return

        self._control_step(current_cartesian)

    # ---------------------------------------------------------
    # CONTROL STEP
    # ---------------------------------------------------------

    def _control_step(self,current_cartesian):

        now = self.get_clock().now()
        cart_cmd = np.zeros(3)
        joint_cmd = np.zeros(6)
        q_d = None

        t_global = (
            now - self.start_time
        ).nanoseconds / 1e9

        if self.control_mode == "PD":

            if self.target_pos_cartesian is None:
                return

            v_cart,error = self._servo_to_cartesian(
                self.target_pos_cartesian,
                current_cartesian
            )
            cart_cmd = v_cart.copy()

            self._publish_twist(v_cart)

            error_log = error

        elif self.control_mode == "CTC":

            if self.q_des_actual is None:
                return

            q_d, qd_d, qdd_d = self._generate_ctc_trajectory()

            if q_d is None:
                return

            qd_cmd,_ = self._ctc_joint(
                q_d,
                qd_d,
                qdd_d
            )
            joint_cmd = qd_cmd.copy()

            self.publish_joint_velocity(qd_cmd)

            if self.target_pos_cartesian is not None:
                error_log = self.target_pos_cartesian - current_cartesian
            else:
                error_log = np.zeros(3)

        # -----------------------------------------------------

        if t_global > 2.0:

            self.rmse_sum += error_log**2
            self.rmse_count += 1

        target_csv = (
            self.target_pos_cartesian
            if self.target_pos_cartesian is not None
            else np.zeros(3)
        )
        self.print_counter += 1
        if self.print_counter >= 10:
            self.print_counter = 0

            rmse = np.sqrt(self.rmse_sum / max(1, self.rmse_count))

            self.get_logger().info(
                f"[{self.control_mode}] "
                f"Actual: ({current_cartesian[0]:.3f}, {current_cartesian[1]:.3f}, {current_cartesian[2]:.3f}) | "
                f"Deseada: ({target_csv[0]:.3f}, {target_csv[1]:.3f}, {target_csv[2]:.3f}) | "
                f"RMSE: {np.linalg.norm(rmse):.4f}"
            )

        q = self.q if self.q is not None else np.zeros(6)
        qd = self.qd if self.qd is not None else np.zeros(6)

        qref = q_d if (self.control_mode == "CTC" and q_d is not None) else np.zeros(6)

        joint_cmd = joint_cmd if self.control_mode == "CTC" else np.zeros(6)

        cart_cmd = cart_cmd if self.control_mode == "PD" else np.zeros(3)

        self.csv_writer.writerow([
            round(t_global,4),

            *target_csv,
            *current_cartesian,
            *cart_cmd,

            *q,
            *qd,
            *qref,

            *joint_cmd,

            self.control_mode,
            1
        ])
        self.csv_file.flush()

        if (
            now - self.last_info_time
        ).nanoseconds > 1e9:

            rmse = np.sqrt(
                self.rmse_sum / max(1,self.rmse_count)
            )

            self.get_logger().info(
                f"Mode: {self.control_mode} | RMSE: {np.linalg.norm(rmse):.4f} m"
            )

            self.last_info_time = now

    # ---------------------------------------------------------
    # CARTESIAN SERVO
    # ---------------------------------------------------------

    def _servo_to_cartesian(self,target,current):

        error = target - current

        d_error = (
            error - self.prev_error_cart
        ) / self.dt

        v = self.kp_cart * error + self.kd_cart * d_error

        v = np.where(
            np.abs(error) > self.epsilon,
            v,
            0.0
        )

        v = np.clip(
            v,
            -self.max_speed,
            self.max_speed
        )

        self.prev_error_cart = error

        return v,error

    # ---------------------------------------------------------
    # CTC CONTROL
    # ---------------------------------------------------------

    def _ctc_joint(self,q_des,qd_des,qdd_des):

        e = q_des - self.q
        ed = qd_des - self.qd

        v = qdd_des + self.Kd_ctc @ ed + self.Kp_ctc @ e

        M = inertia_matrix(self.q)
        C = coriolis_torque(self.q,self.qd)
        G = gravity_torque(self.q)

        tau = M @ v + C + G

        tau = np.clip(
            tau,
            -self.tau_limit,
            self.tau_limit
        )

        qdd_real = np.linalg.solve(
            M,
            tau - C - G
        )

        qd_out = self.qd + qdd_real*self.dt

        return np.clip(qd_out,-2.0,2.0),tau

    # ---------------------------------------------------------
    # TF POSE
    # ---------------------------------------------------------

    def _read_pose(self):

        try:

            trans = self.tf_buffer.lookup_transform(
                "link_base",
                "link_eef",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            return np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])

        except:

            return None

    # ---------------------------------------------------------
    # PUBLISHERS
    # ---------------------------------------------------------

    def _publish_twist(self,v):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"

        msg.twist.linear.x = v[0]
        msg.twist.linear.y = v[1]
        msg.twist.linear.z = v[2]

        self.servo_pub.publish(msg)

    def publish_joint_velocity(self,qdot):

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"

        msg.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        msg.velocities = qdot.tolist()

        msg.duration = 0.01

        self.joint_pub.publish(msg)

    # ---------------------------------------------------------
    # KEYBOARD
    # ---------------------------------------------------------

    def _start_keyboard(self):

        def on_press(key):

            try:

                if key.char == 'p':

                    if self.robot_state == RobotState.RUNNING:
                        self.robot_state = RobotState.PAUSED
                    else:
                        self.robot_state = RobotState.RUNNING

                elif key.char == 'ñ':

                    if self.control_mode == "CTC":
                        self.control_mode = "PD"
                    else:
                        self.control_mode = "CTC"

                    self.get_logger().info(
                        f"Switched to {self.control_mode}"
                    )

            except:
                pass

        self.listener = keyboard.Listener(on_press=on_press)
        self.listener.start()


def main(args=None):

    rclpy.init(args=args)

    node = CircleServoXArmLite6()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()