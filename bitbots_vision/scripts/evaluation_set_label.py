import os
import cv2
import numpy as np
import yaml
import time

class LabelMaskGenerator:
    def __init__(self):
        self._image_size = (600, 800)

    def _generate_horizon_mask_from_vector(self, vector):
        vector = list(vector)
        mask = np.zeros((self._image_size[0], self._image_size[1], 3), dtype=np.uint8)

        vector = [list(pts) for pts in vector]

        vector.append([self._image_size[1] - 1, self._image_size[0] - 0])
        vector.append([0, self._image_size[0] - 1])  # extending the points to fill the space below the horizon
        points = np.array(vector, dtype=np.int32)
        points = points.reshape((1, -1, 2))
        cv2.fillPoly(mask, points, (1, 1, 1))
        return mask

    def iterate_annotations(self, path, annotation_file):
        with open(os.path.join(path, annotation_file), 'r') as f:
            annotations = yaml.load(f)

        for label in annotations['labels']:
            convex_field_boundary = None
            for annotation in label['annotations']:
                if annotation['type'] == "field edge":
                    convex_field_boundary = annotation['vector']
                    break
            if convex_field_boundary:
                mask = self._generate_horizon_mask_from_vector(convex_field_boundary)
                cv2.imwrite(os.path.join(path, label['name']), mask)


if __name__ == "__main__":
    lmg = LabelMaskGenerator()
    lmg.iterate_annotations("/home/florian/Projekt/bitbots/eval_labels/", "eval_labels.yaml")

