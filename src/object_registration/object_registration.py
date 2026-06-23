from rclpy.node import Node
import rclpy
from sklearn.cluster import DBSCAN
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point
import csv
import numpy as np

from object_detection_msgs.msg import ObjectDetectionInfoArray, ObjectDetectionInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker


class ObjectRegistration(Node):
    def __init__(self):
        super().__init__('object_registration')

        # Data storage
        self.odom = None
        self.class_points = {} # key: label name, data: list of detection points belonging to that label

        # Publishers
        ## Marker to put at (supposed) position of umbrella
        self.pub_marker = self.create_publisher(
            Marker,
            '/centroids',
            10
        )

        # Subscribers
        ## TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_object_detection = self.create_subscription(
            ObjectDetectionInfoArray,
            '/detection_info',
            self.cb_detection,
            10
        )

        self.create_subscription(
            Odometry,
            '/state_estimation',
            self.cb_odom,
            10
        )

        # Timer @ 1Hz to DBSCAN and publish centroid markers
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Object Registration Node has been started.')

    def cb_odom(self, msg: Odometry):
        self.odom = msg
    
    def cb_detection(self, msg: ObjectDetectionInfoArray):
        if self.odom is None:
            self.get_logger().warn('Odometry not yet received.')
            return

        for detection in msg.info:
            label = detection.class_id

            # Store the label if not already stored
            if label not in self.class_points.keys():
                self.class_points[label] = []
            # Convert position to map frame
            point = PointStamped()
            point.header.frame_id = "front_camera_optical_frame"
            point.header.stamp = msg.header.stamp
            point.point = detection.position
            try:
                point_map = self.tf_buffer.transform(point, "map")
                self.class_points[label].append(point_map.point)            
            except Exception as e:
                self.get_logger().warn(f'Failed to transform point: {e}')
                continue
    
    def timer_callback(self):
        # Publish markers for each label's centroids
        for label, points in self.class_points.items():
            if len(points) == 0:
                continue
            
            points_array = np.array([[p.x, p.y, p.z] for p in points])
            try:
                clustering = DBSCAN(eps=0.4, min_samples=3).fit(points_array)
                unique_labels = set(clustering.labels_)
            except Exception as e:
                self.get_logger().warn(f'Failed to perform DBSCAN clustering: {e}')
                continue
            centroids = []
            for cluster_label in unique_labels:
                if cluster_label == -1:
                    continue  # Noise
                cluster_points = points_array[clustering.labels_ == cluster_label]
                centroid = np.mean(cluster_points, axis=0)
                centroids.append(centroid)
                
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = label
            marker.id = cluster_label
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0 if label == "umbrella" else 0
            marker.color.g = 1.0 if label == "chair" else 0
            marker.color.b = 1.0 if label == "table" else 0
            for point in centroids:
                po = Point()
                po.x = point[0]
                po.y = point[1]
                po.z = point[2]
                marker.points.append(po)
            self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRegistration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()