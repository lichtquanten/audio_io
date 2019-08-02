#!/usr/bin/env python
"""Plays audio from a wave file."""
import pyaudio
import rospy
from rospywrapper import TopicSource
import sys

from audio_io_msgs.msg import AudioData

def switch_endianness(data, width):
    out = [None] * len(data)
    idx = 0
    for i in xrange(0, len(data), width):
        for j in xrange(width, 0, -1):
            out[idx] = data[i + j - 1]
            idx += 1
    return str(bytearray(out))

def main():
    p = pyaudio.PyAudio()

    input_topic = rospy.get_param('~input_topic')
    threadsafe = rospy.get_param('~threadsafe', False)
    source = TopicSource(input_topic, AudioData, threadsafe=threadsafe)

    with source:
        current_config = None
        stream = None
        for msg, t in source:
            if rospy.is_shutdown():
                break
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
                    input=False,
                    output=True
                )
            if msg.is_bigendian:
                # PyAudio expects little-endian
                msg.data = switch_endianness(
                    data=msg.data,
                    width=msg.sample_width
                )
            stream.write(msg.data)
    p.terminate()

if __name__ == '__main__':
    rospy.init_node('audio_save', anonymous=True)
    main()
