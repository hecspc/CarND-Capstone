from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        model_path = "../trained_model/frozen_inference_graph.pb"
        # the above path is at ros/trained_model parallel to src/tl_dectector
    
        self.detection_graph = tf.Graph()
    
        with self.detection_graph.as_default():
    
            od_graph_def = tf.GraphDef()
    
            with tf.gfile.GFile(model_path, 'rb') as fid:
    
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            # end of with tf.gfile.GFile(model_path, 'rb') as fid:
        # with self.detection_graph.as_default():
        self.session = tf.Session(graph=self.detection_graph)
    
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
    def convert_class_to_color(self, class_value):
        if class_value == 1:
            light_color = TrafficLight.GREEN
            color_label = "GREEN"
        elif class_value == 2:
            light_color = TrafficLight.RED
            color_label = "RED"
        elif class_value == 3:
            light_color = TrafficLight.YELLOW
            color_label = "YELLOW"
        else:
            light_color = TrafficLight.UNKNOWN
            color_label = "UNKNOWN"
        # end of if class_value == 1
        return (light_color, color_label)
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        image_expanded = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num_det) = self.session.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})
    
        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)
    
        # Print class based on best score
    
        light_color = TrafficLight.UNKNOWN
        color_label = "UNKNOWN"
    
        # for i in range(boxes.shape[0]):
        #     if scores[i] >= 0.70:
        #         if classes[i] == 1:
        #             light_color = TrafficLight.GREEN
        #             color_label = "GREEN"
        #         elif classes[i] == 2:
        #             light_color = TrafficLight.RED
        #             color_label = "RED"
        #         elif classes[i] == 3:
        #             light_color = TrafficLight.YELLOW
        #             color_label = "YELLOW"
        #         else:
        #             light_color = TrafficLight.UNKNOWN
        #             color_label = "UNKNOWN"
        #         # end of if classes[i] == 1
        #     # end of if scores[idx] >= 0.70
        # # end of for i in range(boxes.shape[0])
    
        # find index with the max score[index]
        max_score = scores[0]
        max_index = 0
        for i in range(1, boxes.shape[0]):
            if max_score < scores[i]:
                max_score = scores[i]
                max_index = i
            # end of if max_score < scores[i]
        # end of for i in range(1, boxes.shape[0])
    
        light_color, color_label = self.convert_class_to_color(classes[max_index])
        if max_score < 0.7:
            light_color, color_label = TrafficLight.UNKNOWN, UNKNOWN
        # end of if max_score < 0.7
        rospy.loginfo("Traffic Light Color value: %r, label: %s; score: %f" %
                      (light_color, color_label, max_score))
        return light_color
