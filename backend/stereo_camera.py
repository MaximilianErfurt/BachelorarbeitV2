from mono_camera import MonoCamera
class StereoCamera:
    def __init__(self, camera_left:MonoCamera, camera_right:MonoCamera):
        self.camera_left = camera_left
        self.camera_right = camera_right