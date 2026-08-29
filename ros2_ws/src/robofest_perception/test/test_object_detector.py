import unittest

import cv2
import numpy as np

from robofest_perception.object_detector import DetectorConfig, ObjectDetector


class ObjectDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = ObjectDetector(DetectorConfig(
            min_mine_area_px=50.0,
            max_mine_area_px=3000.0,
            min_object_area_px=100.0,
            object_contrast_threshold=12.0,
        ))

    def test_red_marker_is_reported_as_mine(self):
        image = np.full((240, 320, 3), (90, 110, 95), dtype=np.uint8)
        cv2.circle(image, (100, 120), 18, (0, 0, 255), -1)

        detections = self.detector.detect(image)

        self.assertEqual([item.label for item in detections], ["mine"])
        self.assertGreater(detections[0].confidence, 0.8)
        self.assertAlmostEqual(detections[0].pixel_x, 100.0, delta=1.0)
        self.assertAlmostEqual(detections[0].pixel_y, 120.0, delta=1.0)

    def test_unclassified_shape_is_reported_as_object(self):
        image = np.full((240, 320, 3), (90, 110, 95), dtype=np.uint8)
        cv2.rectangle(image, (200, 80), (240, 150), (30, 30, 30), -1)

        detections = self.detector.detect(image)

        self.assertEqual([item.label for item in detections], ["object"])
        self.assertAlmostEqual(detections[0].pixel_x, 220.0, delta=1.0)
        self.assertAlmostEqual(detections[0].pixel_y, 115.0, delta=1.0)

    def test_mine_is_not_reported_again_as_object(self):
        image = np.full((240, 320, 3), (90, 110, 95), dtype=np.uint8)
        cv2.circle(image, (100, 120), 18, (0, 0, 255), -1)
        cv2.rectangle(image, (200, 80), (240, 150), (30, 30, 30), -1)

        detections = self.detector.detect(image)

        self.assertEqual({item.label for item in detections}, {"mine", "object"})
        self.assertEqual(len(detections), 2)

    def test_plain_ground_has_no_detection(self):
        image = np.full((240, 320, 3), (90, 110, 95), dtype=np.uint8)

        self.assertEqual(self.detector.detect(image), [])


if __name__ == "__main__":
    unittest.main()
