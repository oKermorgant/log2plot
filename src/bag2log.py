#!/usr/bin/env python3

import rosbag2_py as rb
from rclpy.serialization import deserialize_message
import os

# obviously in progress


class Bag2log:
    def __init__(self, bag):

        storage = rb.StorageOptions(bag)
        conv = rb.ConverterOptions('cdr','cdr')
        self.reader = rb.SequentialReader()
        self.reader.open(storage, conv)

        self.topics = [topic.name for topic in self.reader.get_all_topics_and_types()]

