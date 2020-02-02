import cv2
import os
import abc
import rospy
try:
    from pydarknet import Detector, Image
except ImportError:
    rospy.logerr("Not able to run Darknet YOLO! Its only executable under python3 with yolo34py or yolo34py-gpu installed.", logger_name="vision_yolo")
import numpy as np
from .candidate import CandidateFinder, Candidate


class YoloHandler():
    """
    Defines an abstract YoloHandler, which runs/manages the YOLO inference.

    Our YOLO is currently able to detect goalpost and ball candidates.
    """
    def __init__(self, config, model_path):
        """
        Initialization of the abstract YoloHandler.
        """
        self._ball_candidates = None
        self._goalpost_candidates = None
        self._image = None
        # Set config
        self.set_config(config)

    def set_config(self, config):
        """
        Set a new config dict, for parameter adjestments

        :param dict: dict with config values
        """
        # Set if values should be cached
        self._caching = config['caching']
        self._nms_threshold = config['yolo_nms_threshold']
        self._confidence_threshold = config['yolo_confidence_threshold']
        self._config = config

    def set_image(self, img):
        """
        Set a image for yolo. This also resets the caches.

        :param image: current vision image
        """
        # Set image
        self._image = img
        # Reset cached stuff
        self._goalpost_candidates = None
        self._ball_candidates = None

    @abc.abstractmethod
    def predict(self):
        """
        Implemented version should run the neural metwork on the latest image. (Cached)
        """
        raise NotImplementedError

    def get_candidates(self):
        """
        Runs neural network and returns results for all classes. (Cached)
        """
        return [self.get_ball_candidates(), self.get_goalpost_candidates()]

    def get_ball_candidates(self):
        """
        Runs neural network and returns results for ball class. (Cached)
        """
        self.predict()
        return self._ball_candidates

    def get_goalpost_candidates(self):
        """
        Runs neural network and returns results for goalpost class. (Cached)
        """
        self.predict()
        return self._goalpost_candidates

class YoloHandlerDarknet(YoloHandler):
    """
    Yolo34py library implementation of our yolo model
    """
    def __init__(self, config, model_path):
        """
        Initialization of the YoloHandlerDarknet

        :param config: vision config dict
        :param model_path: path to the yolo model
        """
        # Define more paths
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        datapath = os.path.join("/tmp/obj.data")
        namepath = os.path.join(model_path, "obj.names")
        # Generates a dummy file for the library
        self._generate_dummy_obj_data_file(namepath)

        self._config = config

        # Setup detector
        self._net = Detector(bytes(configpath, encoding="utf-8"), bytes(weightpath, encoding="utf-8"), 0.5, bytes(datapath, encoding="utf-8"))
        super(YoloHandlerDarknet, self).__init__(config, model_path)

    def _generate_dummy_obj_data_file(self, obj_name_path):
        """
        Generates a dummy object data file.
        In which some meta information for the library is stored.

        :param obj_name_path: path to the class name file
        """
        # Generate file content
        obj_data = "classes = 2\nnames = " + obj_name_path
        # Write file
        with open('/tmp/obj.data', 'w') as f:
            f.write(obj_data)

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self._ball_candidates is None or self._goalpost_candidates is None or not self._caching:
            # Run neural network
            results = self._net.detect(Image(self._image))
            # Init lists
            self._ball_candidates = []
            self._goalpost_candidates = []
            # Go through results
            for out in results:
                # Get class id
                class_id = out[0]
                # Get confidence
                confidence = out[1]
                if confidence > self._confidence_threshold:
                    # Get candidate position and size
                    x, y, w, h = out[2]
                    x = x - int(w // 2)
                    y = y - int(h // 2)
                    # Create candidate
                    c = Candidate(int(x), int(y), int(w), int(h), confidence)
                    # Append candidate to the right list depending on the class
                    if class_id == b"ball":
                        self._ball_candidates.append(c)
                    if class_id == b"goalpost":
                        self._goalpost_candidates.append(c)

class YoloHandlerOpenCV(YoloHandler):
    """
    Opencv library implementation of our yolo model
    """
    def __init__(self, config, model_path):
        """
        Initialization of the YoloHandlerOpenCV

        :param config:
        :param model_path:
        """
        # Build paths
        weightpath = os.path.join(model_path, "yolo_weights.weights")
        configpath = os.path.join(model_path, "config.cfg")
        # Setup neural network
        self._net = cv2.dnn.readNet(weightpath, configpath)
        # Set default state to all cached values
        self._image = None
        super(YoloHandlerOpenCV, self).__init__(config, model_path)

    def _get_output_layers(self):
        """
        Library stuff
        """
        layer_names = self._net.getLayerNames()

        output_layers = [layer_names[i[0] - 1] for i in self._net.getUnconnectedOutLayers()]

        return output_layers

    def predict(self):
        """
        Runs the neural network
        """
        # Check if cached
        if self._ball_candidates is None or self._goalpost_candidates is None or not self._caching:
            # Set image
            blob = cv2.dnn.blobFromImage(self._image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self._net.setInput(blob)
            self._width = self._image.shape[1]
            self._height = self._image.shape[0]
            # Run net
            self._outs = self._net.forward(self._get_output_layers())
            # Create lists
            class_ids = []
            confidences = []
            boxes = []
            self._ball_candidates = []
            self._goalpost_candidates = []
            # Iterate over output/detections
            for out in self._outs:
                for detection in out:
                    # Get score
                    scores = detection[5:]
                    # Ger class
                    class_id = np.argmax(scores)
                    # Get confidence from score
                    confidence = scores[class_id]
                    # First threshold to decrease candidate count and inscrease performance
                    if confidence > self._confidence_threshold:
                        # Get center point of the candidate
                        center_x = int(detection[0] * self._width)
                        center_y = int(detection[1] * self._height)
                        # Get the heigh/width
                        w = int(detection[2] * self._width)
                        h = int(detection[3] * self._height)
                        # Calc the upper left point
                        x = center_x - w / 2
                        y = center_y - h / 2
                        # Append result
                        class_ids.append(class_id)
                        confidences.append(float(confidence))
                        boxes.append([x, y, w, h])

            # Merge boxes
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self._confidence_threshold, self._nms_threshold)

            # Iterate over filtered boxes
            for i in indices:
                # Get id
                i = i[0]
                # Get box
                box = boxes[i]
                # Convert the box position/size to int
                x = int(box[0])
                y = int(box[1])
                w = int(box[2])
                h = int(box[3])
                # Create the candidate
                c = Candidate(x, y, w, h, confidences[i])
                # Append candidate to the right list depending on the class
                class_id = class_ids[i]
                if class_id == 0:
                    self._ball_candidates.append(c)
                if class_id == 1:
                    self._goalpost_candidates.append(c)


class YoloBallDetector(CandidateFinder):
    """
    A ball detector using the yolo neural network.
    This layer connects a single YOLO network with multiple candidate finders for the different classes,
    in this case the ball class.
    """
    def __init__(self, config, yolo):
        """
        Constructor for the YoloBallDetector.

        :param config: The vision config
        :param yolo: An YoloHandler implementation that runs the yolo network
        """
        # Set the yolo network
        self._yolo = yolo
        # Set the config. Not needed at the moment
        self._config = config

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.

        :param image: current vision image
        """
        self._yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found ball candidates
        """
        return self._yolo.get_ball_candidates()

    def compute(self):
        """
        Runs the yolo network
        """
        self._yolo.predict()

class YoloGoalpostDetector(CandidateFinder):
    """
    A goalpost detector using the yolo neural network.
    This layer connects a single YOLO network with multiple candidate finders for the different classes,
    in this case the goalpost class.
    """
    def __init__(self, config, yolo):
        """
        Constructor for the YoloGoalpostDetector.

        :param config: The vision config
        :param yolo: An YoloHandler implementation that runs the yolo network
        """
        self._config = config
        self._yolo = yolo

    def set_image(self, image):
        """
        Set a image for yolo. This is cached.

        :param image: current vision image
        """
        self._yolo.set_image(image)

    def get_candidates(self):
        """
        :return: all found goalpost candidates
        """
        return self._yolo.get_goalpost_candidates()

    def compute(self):
        """
        Runs the yolo network
        """
        self._yolo.predict()
