#!/usr/bin/env python3

import pyaudio
import wave
import numpy as np
import os.path

import rospy

from std_msgs.msg import String

j = 0




def talker():
    print("in talker")

    pub = rospy.Publisher('record',String, queue_size=10)
    print("1")
    rospy.init_node('recording', anonymous=True)
    print("2")
    rate = rospy.Rate(1)
    print("at while")
    while not rospy.is_shutdown():
        msg = String()
        msg.data = record_sound()
        rospy.loginfo(msg.data)
        pub.publish(msg.data)
        rate.sleep() 

#class MinimalPublisher(Node):

 #   def __init__(self):
#        super().__init__('minimal_publisher')
#        self.publisher_ = self.create_publisher(String, 'record', 10)
#        timer_period = 0.5  # seconds
#        self.timer = self.create_timer(timer_period, self.timer_callback)
#        self.i = 0

#    def timer_callback(self):
        
#        msg = String()
#        msg.data = record_sound()
#        self.publisher_.publish(msg)
#        self.get_logger().info('Publishing: "%s"' % msg.data)
#        self.i = j
        
def save_wav_channel(fn, wav, channel):
    '''
    Take Wave_read object as an input and save one of its
    channels into a separate .wav file.
    '''
    # Read data
    nch   = wav.getnchannels()
    depth = wav.getsampwidth()
    wav.setpos(0)
    sdata = wav.readframes(wav.getnframes())

    # Extract channel data (24-bit data not supported)
    typ = { 1: np.uint8, 2: np.uint16, 4: np.uint32 }.get(depth)
    if not typ:
        raise ValueError("sample width {} not supported".format(depth))
    if channel >= nch:
        raise ValueError("cannot extract channel {} out of {}".format(channel+1, nch))
    print ("Extracting channel {} out of {} channels, {}-bit depth".format(channel+1, nch, depth*8))
    data = np.fromstring(sdata, dtype=typ)
    ch_data = data[channel::nch]

    # Save channel to a separate file
    outwav = wave.open(fn, 'w')
    outwav.setparams(wav.getparams())
    outwav.setnchannels(1)
    outwav.writeframes(ch_data.tostring())
    outwav.close()


form_1 = pyaudio.paInt16
chans = 2
samp_rate = 192000
chunk = 8192
record_secs = 2.5
dev_index = 2
wav_output_filename = 'src/py_bearing/src/audiofile/test1.wav'

testingNum = 0
recNum = 0
exists = 0
newpath = 'src/py_bearing/src/audiofile/recGroup%d' % (testingNum)
while exists == 0:
	newpath = 'src/py_bearing/src/audiofile/recGroup%d' % (testingNum)
	if not os.path.exists(newpath):
		os.makedirs(newpath)
		exists = 1
		break
	testingNum = testingNum + 1


def record_sound(args=None):
	global j
	global recNum
	audio = pyaudio.PyAudio()
	#create pyaudio stream

	stream = audio.open(format = form_1, rate = samp_rate, channels = chans, \
				    input_device_index = dev_index, input = True, \
				    frames_per_buffer=chunk)

	print("recording")
	frames = []

	for i in range(0,int((samp_rate/chunk)*record_secs)):
		data = stream.read(chunk)
		frames.append(data)
		    
	print("finishing recording")

	stream.stop_stream()
	stream.close()
	audio.terminate()

	#save the audiuo frames as .wav
	wavefile = wave.open(wav_output_filename, 'wb')
	wavefile.setnchannels(chans)
	wavefile.setsampwidth(audio.get_sample_size(form_1))
	wavefile.setframerate(samp_rate)
	wavefile.writeframes(b''.join(frames))
	wavefile.close()
	pub_file = 'src/py_bearing/src/audiofile/recGroup%d/recNum%d'% (testingNum, recNum)
	
	filename1 = 'src/py_bearing/src/audiofile/recGroup%d/recNum%d_ch1.wav' % (testingNum, recNum)
	filename2 = 'src/py_bearing/src/audiofile/recGroup%d/recNum%d_ch2.wav' % (testingNum, recNum)
	recNum = recNum + 1

	wav = wave.open('src/py_bearing/src/audiofile/test1.wav')
	save_wav_channel(filename1, wav, 0)
	save_wav_channel(filename2, wav, 1)
	j = j+1
	
	print("finish while")
	return pub_file

def main(args=None):
	#print("1")
	#rclpy.init(args=args)
	#minimal_publisher = MinimalPublisher()
	#print("2")
	#rclpy.spin(minimal_publisher)
	
    print("in main")
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	#minimal_publisher.destroy_node()
	#rclpy.shutdown()
	
if __name__ == '__main__':
    print("beforemain")
    main()
