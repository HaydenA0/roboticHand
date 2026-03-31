import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ForwardKinematics(Node):
    def __init__(self):
        super().__init__("forward_kinematics")

        # Robot Geometric Parameters (from Table 1)
        self.L1 = 0.200
        self.L2_offset = 0.065
        self.L3 = 0.410
        self.L4_offset = -0.065
        self.L5 = 0.310
        self.L6_offset = 0.060
        self.L7 = 0.105

        # Subscriber for joint states (sensor_msgs/JointState)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def dh_matrix(self, theta, d, a, alpha):
        """
        Calculates the DH transformation matrix using Equation (2).
        Args:
            theta: Rotation around Z
            d: Translation along Z
            a: Translation along X
            alpha: Rotation around X
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Implements the matrix from page 3 of the PDF
        T = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ]
        ])
        return T

    def compute_forward_kinematics(self, q):
        """
        Computes the full transformation from base to end-effector.
        """
        # DH Parameters Format: (theta, d, a, alpha)
        # alpha is pi/2 (1.5708) for Z-to-X transitions
        pi_2 = np.pi / 2

        dh_params = [
            (q[0], 0.025, 0, pi_2),             # Joint 1
            (q[1], self.L1, 0, pi_2),           # Joint 2
            (q[2], self.L2_offset, 0, pi_2),    # Joint 3
            (q[3], self.L3, 0, pi_2),           # Joint 4
            (q[4], self.L4_offset, 0, pi_2),    # Joint 5
            (q[5], self.L5, 0, pi_2),           # Joint 6
            (q[6], self.L6_offset + self.L7, 0, 0) # Joint 7 (End tool)
        ]

        T = np.eye(4) # Identity matrix (Start at base)
        positions = [np.array([0, 0, 0])] # Track origin

        for params in dh_params:
            T_i = self.dh_matrix(*params)
            T = T @ T_i # Matrix multiplication
            positions.append(T[:3, 3].copy()) # Store current X,Y,Z

        return T, positions

    def joint_state_callback(self, msg):
        """
        Callback triggered whenever joint positions change.
        """
        # Extract joint positions from the message
        # Ensure we only take the 7 arm joints
        q = list(msg.position)
        
        # Calculate Forward Kinematics
        T, positions = self.compute_forward_kinematics(q)

        # Get end-effector position (Last column, first 3 rows of T)
        pos = T[:3, 3]
        
        self.get_logger().info(
            f"End-Effector Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
