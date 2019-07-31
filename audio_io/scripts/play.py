#!/usr/bin/env python
"""Plays audio from a wave file."""
# Fixed, except for switching endianness
import pyaudio
import rospy
from rospywrapper import TopicSource
import sys
from audio.msg import AudioData

def main():
    p = pyaudio.PyAudio()

    input_topic = rospy.get_param('~input_topic')
    threadsafe = rospy.get_param('~threadsafe', False)
    source = TopicSource(input_topic, AudioData, threadsafe=threadsafe)

    with source:
        current_config = None
        for msg, t in source:
            # Get the latest configuration
            msg_config = {
                'width': msg.sample_width,
                'channels': msg.num_channels,
                'rate': msg.sample_rate,
            }
            # Check if configuration changed. Stop stream, if needed, and
            # update configuration.
            if current_config != msg_config:
                if stream is not None:
                    stream.stop_stream()
                    stream.close()
                    stream = None
                current_config = msg_config
            # Open the stream, if has not been started or has been closed
            if stream is None:
                stream = p.open(
                    format=p.get_format_from_width(msg.sample_width),
                    channels=msg.num_channels,
                    rate=msg.sample_rate,
                    output=True
                )
            # PyAudio expects little-endian data. Convert if necessary.
            # if msg.is_bigendian:
            #     data =
            #     msg.data = data
            stream.write(msg.data)
    p.terminate()

if __name__ == '__main__':
    rospy.init_node('audio_save', anonymous = True)
    main()
