import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # State machine for 3 pallets
        self.state = 'FORWARD'
        self.box_detected = False
        self.box_position = Point()
        self.current_pallet = 1  # Track which pallet we're working on (1, 2, or 3)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.fork_cmd_pub = self.create_publisher(String, 'fork_command', 10)
        
        # Subscribers  
        self.create_subscription(Bool, 'box_detected', self.box_callback, 10)
        self.create_subscription(Point, 'box_position', self.position_callback, 10)
        
        # Control loop
        self.create_timer(0.1, self.control_loop)
        self.state_start = None
    
    def box_callback(self, msg):
        self.box_detected = msg.data
    
    def position_callback(self, msg):
        self.box_position = msg
    
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
    
    def control_loop(self):
        twist = Twist()
        
        if self.state == 'FORWARD':
            if self.box_detected:
                # Check alignment - box_position.x is normalized (-1 to 1, 0 is center)
                lateral_offset = self.box_position.x
                distance = self.box_position.z
                
                # If close enough, switch to alignment/pickup
                if distance < 12.0:
                    # Check if we're well-aligned (within threshold)
                    if abs(lateral_offset) < 0.15:  # Well aligned (center ±0.15)
                        self.get_logger().info(f'Pallet {self.current_pallet} reached and aligned. Starting pickup')
                        self.stop_robot()
                        self.state = 'PICKUP'
                        self.state_start = self.get_clock().now()
                    else:
                        # Need to align - move forward slowly while correcting
                        twist.linear.x = 0.15  # Slower approach
                        twist.angular.z = -lateral_offset * 2.0  # Proportional correction (negative because camera x is inverted)
                        self.cmd_vel_pub.publish(twist)
                        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
                            self.get_logger().info(f'offset: {lateral_offset:.2f}, dist: {distance:.1f}')
                else:
                    # Still far away, move forward while making minor corrections
                    twist.linear.x = 0.3
                    twist.angular.z = -lateral_offset * 1.5  # Gentle steering correction
                    self.cmd_vel_pub.publish(twist)
            else:
                # No box detected, just move forward
                twist.linear.x = 0.3
                self.cmd_vel_pub.publish(twist)
        
        elif self.state == 'PICKUP':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 0.5:
                # Lower forks
                self.fork_cmd_pub.publish(String(data='lower'))
            elif elapsed < 0.9:
                # Move forward to insert forks
                twist.linear.x = 0.15
                self.cmd_vel_pub.publish(twist)
            elif elapsed < 1.1:
                self.stop_robot()
            elif elapsed < 2.9:
                # Raise forks
                self.fork_cmd_pub.publish(String(data='raise'))
            else:
                self.stop_robot()
                self.get_logger().info('Pickup complete. Backing up')
                self.state = 'BACKUP'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'BACKUP':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 3.0:
                twist.linear.x = -0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Backed up. Starting reverse turn')
                self.state = 'REVERSE_TURN'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'REVERSE_TURN':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 7.0:
                # Back up AND turn 90°
                twist.linear.x = -0.4
                twist.angular.z = 1.5
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Reverse turn complete. Adjusting position')
                self.state = 'REVERSE_ADJUST'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'REVERSE_ADJUST':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 7.8:
                # Continue backing to balance the path
                twist.linear.x = -0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Position adjusted. Starting forward turn')
                self.state = 'FORWARD_TURN'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'FORWARD_TURN':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 6.5:
                # Drive forward AND turn 90° (completing 180°)
                twist.linear.x = 0.4
                twist.angular.z = 1.5
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Forward turn complete. Driving to conveyor')
                self.state = 'FORWARD_TO_CONVEYOR'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'FORWARD_TO_CONVEYOR':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 2.0:  # Forward to conveyor
                twist.linear.x = 0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('At conveyor. Dropping off pallet')
                self.state = 'DROPOFF'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'DROPOFF':
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 1.5:
                # Lower forks to drop pallet
                self.fork_cmd_pub.publish(String(data='lower'))
            elif elapsed < 4.5:
                # Back away from pallet
                twist.linear.x = -0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                
                # Check if we've completed all 3 pallets
                if self.current_pallet >= 3:
                    self.state = 'DONE'
                else:
                    self.get_logger().info(f'Pallet {self.current_pallet} complete. Returning')
                    self.state = 'REVERSE_FROM_CONVEYOR'
                    self.state_start = self.get_clock().now()
        
        elif self.state == 'REVERSE_FROM_CONVEYOR':
            # Reverse away from conveyor (reverse the forward to conveyor movement)
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 2.0:  # Reverse same amount as FORWARD_TO_CONVEYOR
                twist.linear.x = -0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Reversed from conveyor. Starting reverse turn')
                self.state = 'REVERSE_TURN_RETURN'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'REVERSE_TURN_RETURN':
            # Reverse turn to face perpendicular to pallets (just 90°, shorter than forward turn)
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 4.0:  # Shortened to 4.0s
                # Back up AND turn opposite direction
                twist.linear.x = -0.4
                twist.angular.z = -1.5  # Opposite turn
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Reverse turn complete. Backing up to next pallet area')
                self.state = 'REVERSE_TO_NEXT'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'REVERSE_TO_NEXT':
            # Back up past current pallet to reach next pallet position
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 11.4:  # Back up to pass by pallet and get to next position
                twist.linear.x = -0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Past pallet area. Starting forward turn to next pallet')
                self.state = 'FORWARD_TURN_TO_PALLET'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'FORWARD_TURN_TO_PALLET':
            # Forward turn to face the next pallet (90° turn while going forward)
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 4.0:  # Shortened to 4.0s to match reverse turn
                # Move forward AND turn opposite direction
                twist.linear.x = 0.4
                twist.angular.z = -1.5  # Opposite turn to original reverse turn
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                self.get_logger().info('Turned to face pallet. Moving forward to detect')
                self.state = 'FORWARD_TO_PALLET'
                self.state_start = self.get_clock().now()
        
        elif self.state == 'FORWARD_TO_PALLET':
            # Move forward until we detect the next pallet
            elapsed = (self.get_clock().now() - self.state_start).nanoseconds / 1e9
            
            if elapsed < 3.0:  # Move forward to compensate for BACKUP
                twist.linear.x = 0.3
                self.cmd_vel_pub.publish(twist)
            else:
                self.stop_robot()
                # Move to next pallet and restart detection cycle
                self.current_pallet += 1
                self.get_logger().info(f'Ready for pallet {self.current_pallet}. Starting detection')
                self.state = 'FORWARD'  # Return to forward state to detect next pallet
        
        elif self.state == 'DONE':
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
