from mono_camera import MonoCamera
from image import Image
class StereoCamera:
    def __init__(self, camera_left:MonoCamera, camera_right:MonoCamera):
        self.camera_left = camera_left
        self.camera_right = camera_right
        self.image_left = None
        self.image_right = None

    def take_mono_images(self):
        """
        take the images for blossom finding
        :return:
        """
        self.image_left = Image("left_image", self.camera_left.take_single_image())
        self.image_right = Image("right_image", self.camera_right.take_single_image())


    def find_cut_out_point(self):
        self.image_left.find_blossom()
        self.image_right.find_blossom()