import numpy as np
import pyrealsense2 as rs


class RealSense:
    def __init__(self, width: int, height: int) -> None:
        # Configure depth and color streams
        try:
            self.pipeline = rs.pipeline()
        except:
            self.pipeline = None
        self.config = rs.config()

        self._check_sensors()

        # Enable stream
        self.align_to_color = rs.align(rs.stream.color)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 30)

        self.connect()

        # Load intrinsics
        self.color_intrinsic, self.depth_intrinsic = self._get_intrinsic()

        print("Camera Ready!")

    def connect(self):
        # Start streaming
        self.pipeline.start(self.config)

    def _check_sensors(self):
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        # Check if color sensor exists
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == "RGB Camera":
                found_rgb = True
                break
        if not found_rgb:
            print("The program requires Depth camera with Color sensor")
            exit(0)

    def _get_intrinsic(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_profile = depth_frame.get_profile()
        color_profile = color_frame.get_profile()
        cvs_profile = rs.video_stream_profile(color_profile)
        dvs_profile = rs.video_stream_profile(depth_profile)
        color_params = cvs_profile.get_intrinsics()
        depth_params = dvs_profile.get_intrinsics()
        color_intrinsic = np.array(
            [
                [color_params.fx, 0, color_params.ppx],
                [0, color_params.fy, color_params.ppy],
                [0, 0, 1],
            ]
        )
        depth_intrinsic = np.array(
            [
                [depth_params.fx, 0, depth_params.ppx],
                [0, depth_params.fy, depth_params.ppy],
                [0, 0, 1],
            ]
        )
        return color_intrinsic, depth_intrinsic

    def __del__(self):
        if self.pipeline is not None:
            self.pipeline.stop()

    def get_depth(self, post_process=False) -> np.ndarray:
        if post_process:
            temporal = rs.temporal_filter()
            spatial = rs.spatial_filter()
            for _ in range(10):
                frames = self.pipeline.wait_for_frames()
                frames = self.align_to_color.process(frames)
                depth_frame = frames.get_depth_frame()
                depth_frame = spatial.process(depth_frame)
                depth_frame = temporal.process(depth_frame)
                depth_image = np.asanyarray(depth_frame.get_data())
                return depth_image
        else:
            frames = self.pipeline.wait_for_frames()
            frames = self.align_to_color.process(frames)
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            # dtype is uint16
            return depth_image

    def get_color(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image


if __name__ == "__main__":
    W, H = 1280, 720
    cam = RealSense(W, H)
    d = cam.get_depth()
    e = d.tobytes()
    f = np.frombuffer(e, dtype=np.uint16)
    f = f.reshape(H, W)
    print((d != f).sum())
