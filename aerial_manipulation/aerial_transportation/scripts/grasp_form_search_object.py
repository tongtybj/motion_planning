#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, Point32, Inertia, Vector3,Vector3Stamped, Pose2D
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from aerial_transportation.srv import *
import tf

class TargetObject:

    def init(self):
        rospy.init_node('object_display', anonymous=True)

        self.object_type = -1
        self.inertia = Inertia()
        self.vertices = []
        if not rospy.has_param('~object_type'):
            rospy.logfatal("no config about object_type")
            sys.exit(1)
        else:
            self.object_type = rospy.get_param('~object_type')

        self.odom = Odometry()
        self.odom.pose.pose.orientation.w = 1
        self.yaw = 0

        if self.object_type == ObjectConfigureRequest.CONVEX_POLYGONAL_COLUMN:
            ## param: convex_polygonal_column
            vertex_num = 0
            if not rospy.has_param('~vertex_num'):
                rospy.logfatal("no config about number of vetex")
                sys.exit(1)
            else:
                vertex_num = rospy.get_param('~vertex_num')

                for i in range(vertex_num):
                    vertex = Vector3()
                    if not rospy.has_param('~x_' + str(i + 1)):
                        rospy.logfatal("no config about x value in " + str(i + 1))
                        sys.exit(1)
                    else:
                        vertex.x = rospy.get_param('~x_' + str(i + 1))

                    if not rospy.has_param('~y_' + str(i + 1)):
                        rospy.logfatal("no config about y value in " + str(i + 1))
                        sys.exit(1)
                    else:
                        vertex.y = rospy.get_param('~y_' + str(i + 1))
                        self.vertices.append(vertex)

        if self.object_type == ObjectConfigureRequest.CYLINDER:
            if not rospy.has_param('~radius'):
                rospy.logfatal("no config about radius value")
                sys.exit(1)
            else:
                self.object_radius = Vector3()
                self.object_radius.x = rospy.get_param('~radius')
                self.vertices.append(self.object_radius)

        if not rospy.has_param('~object_mass'):
            rospy.logfatal("no config about object_mass")
            sys.exit(1)
        else:
            self.inertia.m = rospy.get_param('~object_mass')
            self.object_height = rospy.get_param('~object_height', 0.3)
            if self.object_type == ObjectConfigureRequest.CONVEX_POLYGONAL_COLUMN:
                # approximated
                side_len_square = math.pow(self.vertices[1].y - self.vertices[0].y, 2) + math.pow(self.vertices[1].x - self.vertices[0].x, 2)
                self.side_len = math.sqrt(side_len_square)
                self.inertia.ixx = (side_len_square / 4 + self.object_height * self.object_height / 4) / 3 * self.inertia.m
                self.inertia.iyy = self.inertia.ixx
                self.inertia.izz = side_len_square / 6 * self.inertia.m

            if self.object_type == ObjectConfigureRequest.CYLINDER:
                self.inertia.ixx = (self.object_radius.x * self.object_radius.x /16 + self.object_height * self.object_height / 12) * self.inertia.m
                self.inertia.iyy = self.inertia.ixx
                self.inertia.izz = self.object_radius.x * self.object_radius.x /8 * self.inertia.m

        # client
        self.only_pub = rospy.get_param('~only_pub')
        if not self.only_pub:
            rospy.wait_for_service('/target_object_configuration')
            try:
                object_configuration = rospy.ServiceProxy('/target_object_configuration', ObjectConfigure)
                resp = object_configuration(self.object_type, self.inertia, self.vertices)
                print resp.message
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


        self.object_2d_pose_pub = rospy.Publisher("/object/2dpose", Vector3Stamped, queue_size = 1)
        self.odometry_sub = rospy.Subscriber("/object/odom", Odometry, self.odometry_cb)
        self.pose2d_sub = rospy.Subscriber("/object/ground_pose", Pose2D, self.pose2d_cb)

        self.target_object_marker_pub = rospy.Publisher('target_object', Marker, queue_size=10)
        self.polygon_pub = rospy.Publisher('convex_polygonal_column_object', PolygonStamped, queue_size=10)

    def odometry_cb(self, msg):
        self.odom = msg
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def pose2d_cb(self, msg):
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.odom.pose.pose.position.z = 0.15
        self.yaw = msg.theta
        self.odom.pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw))

    def run(self):
        control_frequency = rospy.get_param("~control_frequency", 10)
        rate = rospy.Rate(control_frequency)

        while not rospy.is_shutdown():
            # visualize
            object_marker = Marker()
            object_marker.header.stamp = rospy.get_rostime()
            object_marker.header.frame_id = "world"
            object_marker.ns = "object"
            object_marker.scale.z = 0.3
            object_marker.color.a = 1.0
            object_marker.color.r = 1.0
            object_marker.color.g = 0.0
            object_marker.color.b = 0.0
            object_marker.action = 0
            object_marker.pose = self.odom.pose.pose

            if self.object_type == ObjectConfigureRequest.CONVEX_POLYGONAL_COLUMN:
                # 2D polygon visualization
                object_polygon = PolygonStamped()
                object_polygon.header.frame_id = "object"
                for i in range(len(self.vertices)):
                    vertex = Point32()
                    vertex.x = self.vertices[i].x
                    vertex.y = self.vertices[i].y
                    vertex.z = self.vertices[i].z
                    object_polygon.polygon.points.append(vertex)

                object_polygon.header.stamp = rospy.get_rostime()
                self.polygon_pub.publish(object_polygon)

                # box marker
                object_marker.scale.x = self.side_len
                object_marker.scale.y = self.side_len
                object_marker.type = Marker.CUBE

            if self.object_type == ObjectConfigureRequest.CYLINDER:
                object_marker.scale.x = self.vertices[0].x
                object_marker.scale.y = self.vertices[0].x
                object_marker.type = Marker.CYLINDER

            self.target_object_marker_pub.publish(object_marker)

            # pose 2d
            pose_2d_msg = Vector3Stamped()
            pose_2d_msg.header.stamp = rospy.get_rostime()
            pose_2d_msg.vector.x = self.odom.pose.pose.position.x
            pose_2d_msg.vector.y = self.odom.pose.pose.position.y
            pose_2d_msg.vector.z = self.yaw
            self.object_2d_pose_pub.publish(pose_2d_msg)

            # tf
            br = tf.TransformBroadcaster()
            br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z),
                             (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w),
                             rospy.Time.now(),
                             "object",  "world")

            rate.sleep()

if __name__ == '__main__':
    try:
        target_object = TargetObject()
        target_object.init()
        target_object.run()
    except rospy.ROSInterruptException:
        pass
