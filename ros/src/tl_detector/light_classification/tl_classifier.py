from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self, is_site):
        #TODO load classifier
        # main code source : object_detection_tutorial.ipynb from Google's model-zoo on GitHub
        if is_site:
            PATH_TO_FROZEN_GRAPH = r'/home/workspace/CarND-Capstone/frozen_models/frozen_site_inception/frozen_inference_graph.pb'
        else:
            PATH_TO_FROZEN_GRAPH = r'/home/workspace/CarND-Capstone/frozen_models/frozen_simulator_inception2/frozen_inference_graph.pb'

        self.scores_threshold = 0.25
            
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
                # get the tensors by their names
                self.image_tensor      = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.detection_boxes   = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.detection_scores  = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections    = self.detection_graph.get_tensor_by_name('num_detections:0')
                
            self.sess = tf.Session(graph=self.detection_graph)
                

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.detection_graph.as_default():
            #output_dict = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            #                            feed_dict={self.image_tensor: np.expand_dims(image, 0)})
            (boxes, scores, classes, num_detections) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                                        feed_dict={self.image_tensor: np.expand_dims(image, 0)})
            
        #classes = output_dict['detection_classes'][0]
        #scores  = output_dict['detection_scores'][0]
        # remove 'useless' one-dimensions with 'squeeze' function
        classes = np.squeeze(classes).astype(np.uint8)
        scores  = np.squeeze(scores)
        
        print('Classes (GREEN=1 and RED=2)= ', classes[0], ' - Scores = ', scores[0])
            
        if scores[0] > self.scores_threshold:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.RED
            elif classes[0] == 3:
                return TrafficLight.YELLOW
        
        return TrafficLight.UNKNOWN
