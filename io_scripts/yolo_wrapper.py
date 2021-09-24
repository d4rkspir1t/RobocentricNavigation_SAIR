import numpy as np
import time
import cv2
import os

np.random.seed(42)


class YoloDetector:
    def __init__(self):
        self.labelsPath = os.path.join("../yolo_dir", "coco.names")
        self.LABELS = open(self.labelsPath).read().strip().split("\n")
        self.COLORS = np.random.randint(0, 255, size=(len(self.LABELS), 3), dtype="uint8")

        self. weightsPath = os.path.join("../yolo_dir", "yolov4.weights")
        self.configPath = os.path.join("../yolo_dir", "yolov4.cfg")

        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightsPath)

        self.in_threshold = 0.3
        self.in_confidence = 0.5
        print("[INFO] loaded human detector!")

    def detect_humans(self, image):
        max_id = -1
        # print('[NOTE] oh not, not humans ~screaming in AI~')
        (H, W) = image.shape[:2]
        # print('H, W', H, W)

        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        layerOutputs = self.net.forward(ln)
        end = time.time()

        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > self.in_confidence:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.in_confidence, self.in_threshold)

        # ensure at least one detection exists
        print('%s detector found: %s points in: %s sec.' % ('YOLOv4', len(idxs), end - start))
        # print('%d humans in my sight. Hit one now, harvest the hide, Asimov - I won\'t abide.' % (len(idxs)))
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                if self.LABELS[classIDs[i]] == 'person':
                    if i > max_id:
                        max_id = i
                    # extract the bounding box coordinates
                    (x, y) = (boxes[i][0], boxes[i][1])
                    (w, h) = (boxes[i][2], boxes[i][3])

                    # draw a bounding box rectangle and label on the image
                    color = [int(c) for c in self.COLORS[classIDs[i]]]
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        return image
