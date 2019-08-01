#!/usr/bin/env python
# Working on it
import pyaudio
import rospy
from sys import byteorder

from audio_io_msgs.msg import AudioData
from std_msgs.msg import Header

class InvalidDevice(Exception):
	pass

def device_index_by_name(p, name):
	name = name.lower()
	for idx in xrange(p.get_device_count()):
		n = p.get_device_info_by_index(idx)['name']
		if name in n.lower():
			return idx
	return None

def main():
	p = pyaudio.PyAudio()

	# Get audio configuration
	sample_rate = rospy.get_param('~sample_rate', None)
	num_channels = rospy.get_param('~num_channels', 1)
	sample_width = rospy.get_param('~sample_width', 2)
	format = p.get_format_from_width(sample_width)

	frames_per_buffer = rospy.get_param('~frames_per_buffer', 1024)
	device_index = rospy.get_param('~device_index', None)
	device_name = rospy.get_param('~device_name', None)

	# Find device index
	if device_index is None:
		# Use default device
		if device_name is None:
			device_index = p.get_default_input_device_info()['index']
			# Look-up device by name
		else:
			device_index = device_index_by_name(p, device_name)
			if device_index is None:
				p.terminate()
				raise InvalidDevice('Invalid device name: %s.' % device_name)

	rospy.loginfo("Using device: {}".format(
	p.get_device_info_by_index(device_index)['name'])
	)

	# Use default sample rate, if not provided
	if sample_rate is None:
		sample_rate = p.get_device_info_by_index(device_index)['defaultSampleRate']

	pub_data = rospy.Publisher('~data', AudioData, queue_size=10)

	sample_rate = int(sample_rate)

	# Open pyaudio stream
	stream = p.open(
		input=True,
		output=False,
		format=format,
		channels=num_channels,
		rate=sample_rate,
		frames_per_buffer=frames_per_buffer,
		input_device_index=device_index)

	is_bigendian = (byteorder == 'big')

	while not rospy.is_shutdown():
		data = stream.read(frames_per_buffer, exception_on_overflow=False)
		t = rospy.Time.now()
		msg = AudioData(
			data=data,
			sample_rate=sample_rate,
			num_channels=num_channels,
			sample_width=sample_width,
			is_bigendian=is_bigendian,
		)
		msg.header.stamp = t
		pub_data.publish(msg)

	stream.stop_stream()
	stream.close()
	p.terminate()

if __name__ == '__main__':
	rospy.init_node('capture', anonymous=True)
	main()
