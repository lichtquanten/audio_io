ROS Audio IO
=================

## Overview

This is a ROS package for streaming audio from microphones, playing audio over speakers, and saving audio to files.

## Nodes

### 1. Capture

Uses PyAudio to capture audio from a device.

Example usage:

```bash
rosrun audio_io capture.py __name:=mic
```

#### Published Topics

**~data** (audio_io_msgs/AudioData): Chunks from PyAudio, with configuration information.

#### Parameters

**~device_index** (int, default: default device): Index of the input device. If not set, `device_name` is used.

**~device_name** (int, default: default device): The name of the input device. Can be a substring of the name. Use `arecord -l` to find device names. Used only if device index not set. If not set, default device is used.

**~sample_rate** (int, default: device default): Sample rate (Hz). Device's default sample rate used if not set.

**~sample_width** (int, default: 2): Number of bytes per sample.

**~num_channels** (int, default: 1): Can be mono or stereo.

**~frames_per_chunk** (int, default: 4096): The number of frames pulled from PyAudio buffer at once. Also the number of frames sent in each `AudioData` message.

### 2. Save

Saves an audio stream to a .wav file.

```bash
rosrun audio_io save.py _input_topic:=/mic/data _dst:=~/test.wav
```

#### Published Topics

None

#### Parameters

**~input_topic** (string): A ROS topic publishing `AudioData` messages.

**~dst** (string, default: dst.wav): Absolute or relative path of the output file.

### 2. Save

Plays an audio stream over the default speaker device.

```bash
rosrun audio_io play.py _input_topic:=/mic/data
```

#### Published Topics

None

#### Parameters

**~input_topic** (string): A ROS topic publishing `AudioData` messages.
