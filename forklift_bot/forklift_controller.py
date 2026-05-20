import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ForkliftController(Node):
    def __init__(self):
        super().__init__('forklift_controller')
        
        # Publisher for fork trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/fork_controller/joint_trajectory',
            10
        )
        
        # Subscriber for fork commands
        self.fork_command_sub = self.create_subscription(
            String,
            'fork_command',
            self.fork_command_callback,
            10
        )
        
        # Subscribe to joint states for monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_fork_position = 0.0
        self.target_position = 0.0
        
        # Control loop at 10Hz for publishing trajectory
        self.create_timer(0.1, self.control_loop)
        
        # Status monitoring at 2Hz
        self.create_timer(0.5, self.status_monitor)
    
    def joint_state_callback(self, msg):
        # Monitor current fork position.
        try:
            idx = msg.name.index('fork_lift_joint')
            self.current_fork_position = msg.position[idx]
        except (ValueError, IndexError):
            pass
    
    def control_loop(self):
        # Publish trajectory command for target position.
        msg = JointTrajectory()
        msg.joint_names = ['fork_lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [self.target_position]
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds
        
        msg.points.append(point)
        self.trajectory_pub.publish(msg)
    
    def status_monitor(self):
        # Monitor fork position.
        error = abs(self.target_position - self.current_fork_position)
        if error > 0.02:  # More than 2cm away
            direction = "RAISING" if self.target_position > self.current_fork_position else "LOWERING"
            self.get_logger().info(f'{direction} forks to {self.target_position:.1f}m (current: {self.current_fork_position:.3f}m, error: {error:.3f}m)')
    
    def fork_command_callback(self, msg):
        # Handle fork movement commands.
        command = msg.data.lower()
        
        if command in ['raise', 'up']:
            self.target_position = 0.5
            self.get_logger().info('COMMAND: Raise forks to 0.5m')
        elif command in ['lower', 'down']:
            self.target_position = 0.0
            self.get_logger().info('COMMAND: Lower forks to 0.0m')
        else:
            self.get_logger().warn(f'Unknown command: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
