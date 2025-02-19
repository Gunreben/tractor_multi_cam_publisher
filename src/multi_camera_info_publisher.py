import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class MultiCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_info_publisher')
        # Define camera names; add or adjust as needed.
        self.camera_names = ['side_left', 'rear_left', 'rear_mid', 'rear_right', 'side_right']

        
        # Create a publisher for each camera's camera_info topic.
        self.publishers = {}
        for name in self.camera_names:
            topic = f'/camera/{name}/camera_info'
            self.publishers[name] = self.create_publisher(CameraInfo, topic, 10)
            self.get_logger().info(f'Publisher created for: {topic}')

        # Create a timer to publish at a fixed rate (e.g., 10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Set up a base CameraInfo message.
        self.camera_info = CameraInfo()
        self.setup_camera_info()

    def setup_camera_info(self):
        # Example values based on 1280x720 resolution and a 130° horizontal FOV.
        fx = 298.0
        fy = 298.0
        cx = 640.0
        cy = 360.0

        self.camera_info.width = 1280
        self.camera_info.height = 720
        self.camera_info.distortion_model = "plumb_bob"  # or "equidistant" if appropriate
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]   # Distortion coefficients
        
        # Intrinsic camera matrix (3x3)
        self.camera_info.K = [
            fx,   0.0,  cx,
            0.0,  fy,   cy,
            0.0,  0.0,  1.0
        ]
        
        # Rectification matrix (identity, if no rotation is applied)
        self.camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix (3x4). Often [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0].
        self.camera_info.P = [
            fx,   0.0,  cx,   0.0,
            0.0,  fy,   cy,   0.0,
            0.0,  0.0,  1.0,  0.0
        ]

    def timer_callback(self):
        # Update the header stamp for the current time
        self.camera_info.header.stamp = self.get_clock().now().to_msg()

        # Publish the CameraInfo message on all camera_info topics
        for name, publisher in self.publishers.items():
            self.camera_info.header.frame_id = f'camera_{name}_frame'
            publisher.publish(self.camera_info)
            self.get_logger().info(f'Published camera info on /camera/{name}/camera_info')

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

