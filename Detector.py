import time, cv2, os
from cv2 import waitKey
import tensorflow as tf
import numpy as np

from tensorflow.python.keras.utils.data_utils import get_file

class Detector:
    def __init__(self):
        self.readClasses(classesFilePath="coco.names")
        self.loadModel()

    def readClasses(self, classesFilePath):
        with open(classesFilePath, 'r') as f:
            self.classesList = f.read().splitlines()
 
    def loadModel(self):

        self.modelName = "ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8"
        self.cacheDir = "./pretrained_models"
        print("Loading model " + self.modelName)

        tf.keras.backend.clear_session()
        self.model = tf.saved_model.load(os.path.join(self.cacheDir, "checkpoints", self.modelName, "saved_model"))

        print("Model " + self.modelName + " loaded sucessfully...") 

    def detectBoundingBox(self, frame, threshold = 0.5):

        points = []

        # Create Input
        inputTensor = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
        inputTensor = tf.convert_to_tensor(inputTensor, dtype=tf.uint8)
        inputTensor = inputTensor[tf.newaxis, ...]

        # Execute Model with Input
        detections = self.model(inputTensor)

        # Paramaters received
        dBoxes = detections['detection_boxes'][0].numpy() 
        dClasses = detections['detection_classes'][0].numpy().astype(np.int32)
        dScores = detections['detection_scores'][0].numpy()

        # Shape of frame
        imH, imW, imC = frame.shape

        # Detected Box Index
        dBoxIdx = tf.image.non_max_suppression(dBoxes, dScores, max_output_size=50, 
                iou_threshold=threshold, score_threshold=threshold)

        if len(dBoxIdx) != 0:
            for i in range(len(dBoxIdx)):
                dBox = tuple(dBoxes[i].tolist())
                # dConfidence = round(dScores[i]*100)
                dClass = dClasses[i]

                if dClass == 1: #Class detected is person

                    ymin, xmin, ymax, xmax = dBox

                    xmin = int(xmin*imW) #+ x1
                    xmax = int(xmax*imW) #+ x1
                    ymin = int(ymin*imH) #+ y1
                    ymax = int(ymax*imH) #+ y1

                    points.append([xmin, xmax, ymin, ymax])

        return points
