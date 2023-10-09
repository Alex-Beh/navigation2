import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import yaml


class NavigateThroughPosesExample(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_example')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.wp_navigation_markers_pub_ = self.create_publisher(MarkerArray, "/waypoints", qos_profile)

        poses_header = Header()
        poses_header.stamp = self.get_clock().now().to_msg()
        poses_header.frame_id = 'map'

        poses_sequence = []
        with open('cycle.yaml', 'r') as file:
            waypoint_lists = yaml.safe_load(file)

        for pose in waypoint_lists:
            print(pose, ': ', waypoint_lists[pose])
            new_pose = PoseStamped()
            new_pose.header = poses_header
            new_pose.pose.position.x = waypoint_lists[pose]['position']['x']
            new_pose.pose.position.y = waypoint_lists[pose]['position']['y']
            new_pose.pose.orientation.w = waypoint_lists[pose]['orientation']['w']
            new_pose.pose.orientation.z = waypoint_lists[pose]['orientation']['z']
            poses_sequence.append(new_pose)

        self.client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses'
        )

        print("wait for server")
        self.client.wait_for_server()
        print("Server is ready")
        self.publish_markers(poses_header, poses_sequence)
        self.send_goals(poses_sequence)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback--> number_of_poses_remaining: {feedback.number_of_poses_remaining}')

    def send_goals(self, poses_sequence):
        print('Sending Goals')

        goal_msg = NavigateThroughPoses.Goal(poses=poses_sequence)

        future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        print("Done")

        if future.result() is not None:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Hooray, reached the desired pose')
            else:
                self.get_logger().error('The base failed to reach the desired pose')
        else:
            self.get_logger().error('Failed to get response from server')

    def publish_markers(self, header, poses_sequence):
        marker_array = MarkerArray()
        
        marker_id = 0
        for i, pose in enumerate(poses_sequence):
            # Draw a green arrow at waypoint pose
            arrow_marker = Marker()
            arrow_marker.header = header
            # arrow_marker.ns = 'waypoints'
            arrow_marker.id = marker_id
            marker_id += 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = pose.pose
            arrow_marker.scale.x = 0.3
            arrow_marker.scale.y = 0.05
            arrow_marker.scale.z = 0.02
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 255.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0
            arrow_marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            arrow_marker.frame_locked = False
            marker_array.markers.append(arrow_marker)
            
            # Draw a red circle at waypoint pose
            circle_marker = Marker()
            circle_marker.header = header
            # circle_marker.ns = 'waypoints'
            circle_marker.id = marker_id
            marker_id += 1
            circle_marker.type = Marker.SPHERE
            circle_marker.action = Marker.ADD
            circle_marker.pose = pose.pose
            circle_marker.scale.x = 0.05
            circle_marker.scale.y = 0.05
            circle_marker.scale.z = 0.05
            circle_marker.color.r = 255.0
            circle_marker.color.g = 0.0
            circle_marker.color.b = 0.0
            circle_marker.color.a = 1.0
            circle_marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            circle_marker.frame_locked = False
            marker_array.markers.append(circle_marker)
                    
            # Draw the waypoint number
            marker_text = Marker()
            marker_text.header = header
            # marker_text.ns = 'waypoints'
            marker_text.id = marker_id
            marker_id += 1
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.ADD
            marker_text.pose = pose.pose
            marker_text.scale.x = 0.07
            marker_text.scale.y = 0.07
            marker_text.scale.z = 0.07
            marker_text.color.r = 0.0
            marker_text.color.g = 255.0
            marker_text.color.b = 0.0
            marker_text.color.a = 1.0
            marker_text.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
            marker_text.frame_locked = False
            marker_text.text = 'wp_'+str(i)
            marker_array.markers.append(marker_text)
            
        if(len(marker_array.markers)):
            clear_all_marker = Marker()
            clear_all_marker.action = Marker.DELETEALL
            clear_all_markers = MarkerArray()
            clear_all_markers.markers.append(clear_all_marker)
            self.wp_navigation_markers_pub_.publish(clear_all_markers)
      
        self.wp_navigation_markers_pub_.publish(marker_array)
      
def main(args=None):
    rclpy.init(args=args)
    navigator = NavigateThroughPosesExample()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
