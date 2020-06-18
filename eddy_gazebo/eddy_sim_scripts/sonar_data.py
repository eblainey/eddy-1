import rosbag, cv2, sys, struct, itertools
import numpy as np
from PIL import Image
from typing import List
import matplotlib.pyplot as plt
import numpy.core._multiarray_umath


def get_image():
    bag = rosbag.Bag('eddy.bag', 'r')
    num = bag.get_message_count(topic_filters=['image_view/output'])
    image = bag.read_messages(topics='[image_view/output]')
    for i in range(num):
        topic, msg, t = image
        img = np.fromstring(image.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.width)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        img.save('sonar_history.png')


def get_array_dim():
    img = Image.open('sonar_history.png')
    array = np.array(img)
    rospyloginfo(array.shape)


def get_b_array():
    bag = rosbag.Bag('eddy.bag', 'r')
    num = bag.get_message_count(topic_filters=['eddy/sonar/profile'])
    profile = bag.read_messages(topics='[eddy/sonar/profile]')
    for i in range(num):
        topic, msg, t = profile
        b_array.append(msg, dtype=uint8)
    return b_array


class Waterfall:

    def __init__(self):
        self.array = np.zeros([200, 200, 3], np.uint8)
        self.array = np.array(b_array, np.uint8)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplots()
        self.draw()
