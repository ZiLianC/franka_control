import json
import time
import rospy
import numpy as np
import quaternion
from panda_robot import PandaArm

import http.server as BaseHTTPServer
from urllib.parse import urlparse, parse_qs


class PandaController:
    def __init__(self):
        rospy.init_node("panda_control_python")

        self._arm = PandaArm()
        self._gripper = self._arm.get_gripper()
        self._gripper.open()
        # arm.move_to_neutral()

    def robot_name(self):
        return self._arm.name

    def move_to_neutral(self):
        self._arm.move_to_neutral()

    def get_gripper_pose(self):
        endpoint_pose = self._arm.endpoint_pose()
        return endpoint_pose

    def get_joint_angles(self):
        return self._arm.joint_angles()  # dict, str:float

    def send_cartesian_pose(self, pos, ori):
        pos = np.array(pos)

        ori = np.array(ori)
        ori = ori / np.linalg.norm(ori)
        ori = np.quaternion(*ori)  # w,x,y,z

        #self._arm.move_to_cartesian_pose(pos, ori)
        flag, joint_pos = self._arm.inverse_kinematics(pos, ori)
        if flag:
            self._arm.move_to_joint_position(joint_pos)
            time.sleep(2)
        else:
            print('cannot calculate inverse_kinematics. 1')

    def send_joint_angles(self, joint_angles:list):
        print(joint_angles)
        self._arm.move_to_joint_position(joint_angles)
        time.sleep(2)

    def send_gripper_command(self, width):
        self._gripper.move_joints(width)  # width is in meter

    def close_gripper(self):
        self._gripper.close()

    def open_gripper(self):
        self._gripper.open()


panda_controller = PandaController()


class PandaHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        print('command:', path)
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write("Hello from Panda: {}".format(panda_controller.robot_name()))
        else:
            target = "_get" + "_".join(self.path.split("/"))
            print(target)
            if not hasattr(self, target):
                self._handle_404()
            else:
                self.send_response(200)
                self.send_header("Content-type", "application/numpy")
                self.end_headers()
                m = getattr(self, target)
                content = m()
                self.wfile.write(content)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/pose":
            self._post_pose()
        elif path == "/gripper":
            self._post_gripper()
        elif path == "/joint_angles":
            self._post_joint_angles()
        else:
            self._handle_404()

    def _get_pose(self):
        pose = panda_controller.get_gripper_pose()
        pose_dict = {
            'tx': pose['position'][0],
            'ty': pose['position'][1],
            'tz': pose['position'][2],
            'qx': pose['orientation'].x,
            'qy': pose['orientation'].y,
            'qz': pose['orientation'].z,
            'qw': pose['orientation'].w
        }
        return json.dumps(pose_dict).encode()

    def _get_joint_angles(self):
        joint_angles = panda_controller.get_joint_angles()  # dict
        return json.dumps(joint_angles).encode()

    def _get_stop(self):
        """
        stop the robot
        """
        # DG.stop_gripper()
        return "Do nothing".encode()

    def _get_close(self):
        """
        stop the robot
        """
        panda_controller.close_gripper()
        return "Close gripper".encode()

    def _get_open(self):
        """
        stop the robot
        """
        panda_controller.open_gripper()
        return "Open gripper".encode()

    def _get_neutral(self):
        panda_controller.move_to_neutral()
        return "Move Panda to neutral position".encode()

    def _post_pose(self):
        # receive target pose and control the robot
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode()
        p = parse_qs(post_data)
        tx, ty, tz = float(p['tx'][0]), float(p['ty'][0]), float(p['tz'][0])
        qx, qy, qz, qw = float(p['qx'][0]), float(p['qy'][0]), float(p['qz'][0]), float(p['qw'][0])
        print('target cartesian pose: {}, {}, {}, {}, {}, {}, {}'.format(tx, ty, tz, qx, qy, qz, qw))
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write("Moving the gripper to {}, {}, {}, {}, {}, {}, {}".format(tx, ty, tz, qx, qy, qz, qw).encode())
        panda_controller.send_cartesian_pose(pos=[tx, ty, tz], ori=[qw, qx, qy, qz])

    def _post_joint_angles(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode()
        p = parse_qs(post_data)

        joint_angles = []
        for p_k in p:
            joint_angles.append(float(p[p_k][0]))
        print(joint_angles)
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write("Moving the gripper to joint angles {}, {}, {}, {}, {}, {}, {}".format(joint_angles[0], joint_angles[1], 
        joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6]).encode())
        panda_controller.send_joint_angles(joint_angles)

    def _post_gripper(self):
        # receive target width and control the gripper
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode()
        p = parse_qs(post_data)
        width = float(p['width'][0])
        print(width)
        if 0.0 <= width <= 0.15:
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write("Controlling the gripper to {}".format(width).encode())
            panda_controller.send_gripper_command(width)
        else:
            self.send_response(500)
            self.send_header("Content-type", "text/plain")
            self.end_headers()
            self.wfile.write("width must be in [0.0, 1.0], but got {}".format(width).encode())

    def _handle_404(self):
        self.send_response(404)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write("404 Not Found: {}".format(self.path))


httpd = BaseHTTPServer.HTTPServer(("0.0.0.0", 8001), PandaHandler)
print("Started Panda Http Server on 0.0.0.0:8001")
httpd.serve_forever()
