#!/usr/bin/env python
"""Saves audio to a wave file."""
import rospy
from rospywrapper import TopicSource
import wave

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
    input_topic = rospy.get_param('~input_topic')
    threadsafe = rospy.get_param('~threadsafe', False)
    dst = rospy.get_param('~dst', 'out.wav')

    source = TopicSource(input_topic, AudioData)
    wf = None
    with source:
        for msg, t in source:
            if not wf:
                wf = wave.open(dst, 'wb')
                wf.setnchannels(msg.num_channels)
                wf.setframerate(msg.sample_rate)
                wf.setsampwidth(msg.sample_width)
            if msg.is_bigendian:
                # Wave files are little-endian
                msg.data = switch_endianness(
                    data=msg.data,
                    width=msg.sample_width
                )
            wf.writeframes(msg.data)

if __name__ == '__main__':
    rospy.init_node('save', anonymous=True)
    main()
