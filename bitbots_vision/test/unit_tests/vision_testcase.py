import os
import cv2
from bitbots_test.test_case import TestCase


class VisionTestCase(TestCase):
    def __init__(self, methodName='runTest'):
        self._images = self._get_images()
        super().__init__(methodName)

    def _get_images(self):
        build_dir = os.getenv("CATKIN_BUILD_BITBOTS_VISION")
        image_names = [
            "81-test_nagoya_game_d_00095.png",
            "img_fake_cam_000800.PNG",
        ]
        images = {}
        for image_name in image_names:
            path = os.path.join(build_dir, image_name)
            image = cv2.imread(path)
            self.assertIsNotNone(image, msg=f"Image could not be read: '{path}'")
            images[image_name] = image
        return images

    def test_foo(self):
        self.assertTrue(True)

# class PytorchTestCase(VisionTestCase):
#     def test_foo(self):
#         cv2.imshow("", self._images[0])
#         cv2.waitKey()
#         self.assertTrue(False, f"########################{self._images.keys()}")
