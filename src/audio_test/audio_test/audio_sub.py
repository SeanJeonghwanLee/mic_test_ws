# Standard libraries
import os, sys
import sounddevice as sd
import soundfile as sf
import numpy as np
assert np
from datetime import datetime
from queue import Queue
from loguru import logger
import uuid
import time

# ROS2 libraries
import rclpy
from rclpy.node import Node
from audio_msgs.msg import AudioInfo, AudioData


class AudioSubscriber(Node):

    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(AudioData, 
                                                     'audio_data', 
                                                     self.audio_callback, 
                                                     10)
        self.subscription_info = self.create_subscription(AudioInfo, 
                                                          'audio_info', 
                                                          self.audio_info_callback, 
                                                          10)
        self.subscription # prevent unused variable warning

        # Configure Loguru to set default verbosity to INFO
        logger.remove()  # Remove the default handler
        logger.add(sys.stderr, level="INFO")  # Add a handler to stderr with INFO level


        # Configure loguru to write logs to a file
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # logger.add(f"logfile_{current_time}.txt")
        
        # Sleep 2 seconds for launch file
        time.sleep(5)
        logger.info("Wait 2 seconds for launch file")

    def audio_callback(self, msg:AudioData) -> None:
        logger.debug(f"Received audio data: {msg.data}")
        self.get_logger().info(f"Received audio data")

    def audio_info_callback(self, msg:AudioInfo) -> None:
        logger.info(f"Received audio info: {msg.uuid}")
        self.get_logger().info(f"Received audio info: {msg.uuid}")

def main(args=None):
    rclpy.init(args=args)

    audio_subscriber = AudioSubscriber()

    rclpy.spin(audio_subscriber)

    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()