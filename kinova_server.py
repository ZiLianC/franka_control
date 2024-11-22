"""
@Usage  :   
    $ cd workspace
    $ source devel/setup.bash
    $ python3 kinova_server.py

"""
import math
import time
import json
import http.server as BaseHTTPServer
from urllib import parse as urlparse
import argparse
import rospy
from kortex_driver.msg import *
from kortex_driver.srv import *


class Controller:
    def __init__(self):

        rospy.init_node("kinova_gen3_controller")
        # Get node params
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.degrees_of_freedom = rospy.get_param(
            "/" + self.robot_name + "/degrees_of_freedom", 7
        )
        self.is_gripper_present = rospy.get_param(
            "/" + self.robot_name + "/is_gripper_present", False
        )
        rospy.loginfo("Using robot_name " + self.robot_name)

        # Init the action topic subscriber
        self.action_topic_sub = rospy.Subscriber(
            "/" + self.robot_name + "/action_topic",
            ActionNotification,
            self.cb_action_topic,
        )
        self.last_action_notif_type = None

        read_action_full_name = "/" + self.robot_name + "/base/read_action"
        rospy.wait_for_service(read_action_full_name)
        self.read_action_srv = rospy.ServiceProxy(read_action_full_name, ReadAction)

        # Service for get gripper pose
        get_gripper_pose_srv_name = (
            "/" + self.robot_name + "/base/get_measured_cartesian_pose"
        )
        rospy.wait_for_service(get_gripper_pose_srv_name)
        self.get_gripper_pose_srv = rospy.ServiceProxy(
            get_gripper_pose_srv_name, GetMeasuredCartesianPose
        )

        # Service for send gripper command
        send_gripper_command_name = "/" + self.robot_name + "/base/send_gripper_command"
        rospy.wait_for_service(send_gripper_command_name)
        self.send_gripper_command_srv = rospy.ServiceProxy(
            send_gripper_command_name, SendGripperCommand
        )

        # service for stop gripper
        stop_gripper_srv_name = "/" + self.robot_name + "/base/stop"
        rospy.wait_for_service(stop_gripper_srv_name)
        self.stop_gripper_srv = rospy.ServiceProxy(stop_gripper_srv_name, Stop)

        # Service for execute action(move gripper to pointed pose)
        execute_action_srv_name = "/" + self.robot_name + "/base/execute_action"
        rospy.wait_for_service(execute_action_srv_name)
        self.execute_action_srv = rospy.ServiceProxy(
            execute_action_srv_name, ExecuteAction
        )

        # Service for get joint angles
        get_joint_angles_srv_name = "/" + self.robot_name + "/base/get_measured_joint_angles"
        rospy.wait_for_service(get_joint_angles_srv_name)
        self.get_joint_angles_srv = rospy.ServiceProxy(
            get_joint_angles_srv_name, GetMeasuredJointAngles
        )

        validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def FillCartesianWaypoint(
        self,
        new_x,
        new_y,
        new_z,
        new_theta_x,
        new_theta_y,
        new_theta_z,
        blending_radius,
    ):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = (
            CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        )
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def get_gripper_pose(self):
        req = GetMeasuredCartesianPoseRequest()
        try:
            res = self.get_gripper_pose_srv(req)
            output = res.output
            pose = [
                output.x,
                output.y,
                output.z,
                output.theta_x / 180.0 * math.pi,
                output.theta_y / 180.0 * math.pi,
                output.theta_z / 180.0 * math.pi,
            ]
            return pose
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetMeasuredCartesianPose")
            return False
    
    def get_joint_angles(self):
        """
        Joint Angle is represented by Angle system
        """
        req = GetMeasuredJointAnglesRequest()
        try:
            res = self.get_joint_angles_srv(req)
            output = res.output
            joint_angles = [
                output.joint_angles[0].value,
                output.joint_angles[1].value,
                output.joint_angles[2].value,
                output.joint_angles[3].value,
                output.joint_angles[4].value,
                output.joint_angles[5].value,
                output.joint_angles[6].value,
            ]

            return joint_angles
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetMeasuredJointAngles")
            return False

    def get_gripper_width(self):
        try:
            output = rospy.wait_for_message("/" + self.robot_name + "/joint_states", JointState)
            return output
        except rospy.ServiceException:
            rospy.logerr("Failed to call GetMeasuredGripperMovement")
            return False

    def stop_gripper(self):
        req = StopRequest()
        res = self.stop_gripper_srv(req)

    def send_gripper_command(self, value):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")
        # Call the service
        try:
            self.send_gripper_command_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def send_cartesian_pose(self, pose):
        """
        pose: pose in robot base coordination
        """
        self.last_action_notif_type = None
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        x, y, z = pose[0], pose[1], pose[2]
        theta_x, theta_y, theta_z = (
            pose[3] / math.pi * 180.0,
            pose[4] / math.pi * 180.0,
            pose[5] / math.pi * 180.0,
        )

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(x, y, z, theta_x, theta_y, theta_z, 0)
        )
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        try:
            self.execute_action_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False

    def send_joint_angles(self, angles):
        "The angle is expressed in radians"
        self.last_action_notif_type = None
        for i in range(len(angles)):
            angles[i] = angles[i] / math.pi * 180.0
        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()
        angularWaypoint.angles = angles


        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False
        
        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30
        
        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration
            
            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False
            
            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Send the angles
        rospy.loginfo("Sending the robot angles...")
        try:
            self.execute_action_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

rb = Controller()

class KinovaHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse.urlparse(self.path).path
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write("Hello from Kinova: {}".format(rb.robot_name))
        else:
            target = "_get" + "_".join(self.path.split("/"))
            if not hasattr(self, target):
                self._handle_404()
            else:
                m = getattr(self, target)
                m()

    def do_POST(self):
        path = urlparse.urlparse(self.path).path
        if path == "/pose":
            self._post_pose()
        elif path == "/gripper":
            self._post_gripper()
        elif path == "/joint_angles":
            self._post_joint_angles()
        else:
            self._handle_404()

    def _get_joint_angles(self):
        joint_angles = rb.get_joint_angles()
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(joint_angles).encode())


    def _get_pose(self):
        pose = rb.get_gripper_pose()
        pose = {
            "tx": pose[0],
            "ty": pose[1],
            "tz": pose[2],
            "rx": pose[3],
            "ry": pose[4],
            "rz": pose[5],
        }
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(pose).encode())

    def _get_stop(self):
        """
        stop the robot
        """
        rb.stop_gripper()
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write("Kinova Stopped.".encode())

    def _get_gripper_width(self):
        width = rb.get_gripper_width()
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(width).encode())

    def _post_pose(self):
        # receive target pose and control the robot
        content_length = int(self.headers["Content-Length"])
        post_data = self.rfile.read(content_length).decode()
        p = urlparse.parse_qs(post_data)

        if not self._check_pose(p):
            self.send_response(500)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(("invalid pose values, got {}".format(p)).encode())
        else:
            tx, ty, tz = float(p["tx"][0]), float(p["ty"][0]), float(p["tz"][0])
            rx, ry, rz = float(p["rx"][0]), float(p["ry"][0]), float(p["rz"][0])
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(
                ("Moving the gripper to {},{},{},{},{},{}".format(tx, ty, tz, rx, ry, rz)).encode()
            )
            rb.send_cartesian_pose([tx, ty, tz, rx, ry, rz])

    def _post_joint_angles(self):
        # receive target pose and control the robot
        content_length = int(self.headers["Content-Length"])
        angles_data = self.rfile.read(content_length).decode()
        p = urlparse.parse_qs(angles_data)
        joint1, joint2, joint3, joint4, joint5, joint6, joint7 = \
        float(p["joint1"][0]), float(p["joint2"][0]), float(p["joint3"][0]),float(p["joint4"][0]),float(p["joint5"][0]),float(p["joint6"][0]),float(p["joint7"][0])
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(
            ("Moving the gripper to {},{},{},{},{},{},{}".format(joint1, joint2, joint3, joint4, joint5, joint6, joint7)).encode()
        )
        rb.send_joint_angles([joint1, joint2, joint3, joint4, joint5, joint6, joint7])

    def _post_gripper(self):
        # receive target width and control the gripper
        content_length = int(self.headers["Content-Length"])
        post_data = self.rfile.read(content_length).decode()
        p = urlparse.parse_qs(post_data)
        width = float(p["width"][0])

        if width >= 0.0 and width <= 1.0:
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(("Controlling the gripper to {}".format(width)).encode())
            rb.send_gripper_command(width)
        else:
            self.send_response(500)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write(("width must be in [0.0, 1.0], but got {}".format(width)).encode())

    def _handle_404(self):
        self.send_response(404)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(("404 Not Found: {}".format(self.path)).encode())

    def _check_pose(self, pose):
        """
        check whether the pose is valid
        """
        return len(pose) == 6

parser = argparse.ArgumentParser()
parser.add_argument("--host", type=str, default="0.0.0.0")
parser.add_argument("-p", "--port", type=int, default=8000)
args = parser.parse_args()

addr = args.host
port = args.port
httpd = BaseHTTPServer.HTTPServer((addr, port), KinovaHandler)
print("Started Kinova Http Server on {}:{}".format(addr, port))
httpd.serve_forever()
