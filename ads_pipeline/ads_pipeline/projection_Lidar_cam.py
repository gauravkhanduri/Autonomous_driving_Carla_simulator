import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import cv2
import ros2_numpy as rnp
import numpy as np
from cv_bridge import CvBridge
import os


from ads_pipeline.lidar_projection import projection_lidar_to_image, overlay_projection, build_intrinsic_matrix, build_extrinsic, colorize_depth


#from message_filters import ApproximateTimeSynchronizer, Subscriber


class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.get_logger().info('Projection Node has been started')
        self.latest_lidar = None
        self.latest_cam = None
        self.bridge = CvBridge()


        self.K = build_intrinsic_matrix(image_w=1920, image_h=1080, fov_deg=90.0)
        self.T = build_extrinsic(lidar_loc=(0.0,0.0,2.5), camera_loc=(1.5, 0.0, 2.4))



        # Initialize subscriber for LiDAR and camera 
        # Topics Names: /carla/camera/rgb/image; /carla/lidar/points

        self.lidar_sub = self.create_subscription(PointCloud2,
                                                '/carla/lidar/points'
                                                ,self.lidar_callback,10)
        self.lidar_sub # prevent unused variable warning

        self.cam_sub = self.create_subscription(Image,
                                                '/carla/camera/rgb/image',
                                                self.camera_callback,
                                                10)
        self.cam_sub # prevent unused variable warning

    def lidar_callback(self,msg):
        self.latest_lidar = msg
        


    def camera_callback(self,msg):
        self.latest_cam = msg
        if(self.latest_lidar is not None and self.latest_cam is not None):
            """
            Pointcloud manipulation is updated ros2_numpy.pointcloud2 to be compatible with ROS2.
            It now gives a structured numpy array with the fields x, y, z, rgb, intensity. 
            The rgb field is a 3-tuple of uint8 values. The intensity field is a float32 value.
            The x, y, z fields are float32 values
            """
     

            data = rnp.numpify(self.latest_lidar)
            point_xyz_ = data['xyz']
            # CARLA to camera coordinate conversion
            point_xyz_cam = np.column_stack([
            point_xyz_[:, 1],    # CARLA Y → camera X
            -point_xyz_[:, 2],   # CARLA -Z → camera Y
            -point_xyz_[:, 0],   # CARLA -X → camera Z (forward)
            ])


            pixels, depths = projection_lidar_to_image(points_xyz=point_xyz_cam, K=self.K, 
                                                       T_cam_lidar=np.eye(4),
                                                       image_w=1920,image_h=1080)
            # self.get_logger().info(f'depths shape: {depths.shape}, dtype: {depths.dtype}')
            if len(depths) == 0:
                return

            colors = colorize_depth(depths, max_depth=50.0)
            rgb = self.bridge.imgmsg_to_cv2(self.latest_cam, 'rgb8')
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            overlay = overlay_projection(image_bgr=bgr, pixels=pixels, colors=colors )
            path = os.path.expanduser('~/results/projection_result.png')

            cv2.imwrite(path,overlay)


            
            

def main(args=None):
      rclpy.init(args=args)

      projection = ProjectionNode()

      rclpy.spin(projection)

      projection.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()      