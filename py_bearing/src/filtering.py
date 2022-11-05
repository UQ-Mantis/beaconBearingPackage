#!/usr/bin/env python3

from __future__ import print_function
import scipy.io.wavfile as wavfile
import scipy
import scipy.fftpack
import math
import pyaudio
import wave
import scipy
from scipy.io import wavfile
import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt
from scipy.signal import blackman
from scipy import signal as sig
import rospy
import time
from std_msgs.msg import String

################################################

#change this one 
#BEACON signal frequency 
signal_freq = 22000
#
#
###############################################
oldMsg = "Hello world"
pub = rospy.Publisher('bearing', String, queue_size=10)
start_time = time.time()

def callback(data):
    rospy.loginfo("%s", data.data)
    global oldMsg

    if data.data != oldMsg :
               #print(1)
               msgBearing = String()
               oldMsg = data.data
               #print(2)
               bAngle = angle_loc(data.data)
               #print(msgBearing)
               nan = math.isnan(bAngle)
               #print(5)
               #print(nan)
               #pub.publish('publishing works')
               if math.isnan(bAngle) != True and bAngle != 1000:
                   msgBearing.data = "bearing is: %d" % bAngle
                   pub.publish(msgBearing)

def listener_publisher():

    rospy.init_node('listener_publisher', anonymous=True)
    rospy.Subscriber("record",String,callback)
    rospy.spin()

# performs the fast fourier trasnform on the given signal 
# inputs: sample rate, signal data
# ouput: number of samples	       
def signal_fft(fs_rate, signal):
    #print ("Frequency sampling", fs_rate)
    l_audio = len(signal.shape)
    #print ("Channels", l_audio)
    if l_audio == 2:
        signal = signal.sum(axis=1) / 2
    N = signal.shape[0]
    #print ("Complete Samplings N", N)

    return N


# passes the given signal (sigData) through a bandpass filter 
# inputs: sample rate, signal data, frequency pass band, frequency stop band, passband gain, stopband gain
# output: filtered signal, number of samples
def bandPassFilter(sr, sigData, wp, ws, gpass, gstop):

    samples = signal_fft(sr, sigData)

    order, Wn =sig.buttord(wp, ws, gpass, gstop, analog=False, fs=sr)
    sos = sig.butter(order ,wp, 'bandpass', fs=sr, output='sos')
    bpFiltered = sig.sosfilt(sos, sigData)

    samples = signal_fft(sr, bpFiltered)
    return bpFiltered, samples

#averages the array given 
def average(array):
    return sum(array)/len(array)

# detects if the signal is an outlier (false positive), averages next 3000 samplesto check if signal length is long enough 
# inputs: signal start time sample number, complete signal data
# outPut: average of the 700 samples
def is_sig_outlier(index, data, sampleNum):
    avAry = [0]*700 #initialise array 
    for n in range(700):   #loop through samples and add them to array
        i = n + index
        if i >= sampleNum:
            #print("broken aVary")
            break
        avAry[n] = abs(data[i])

    av = average(avAry)
    return av
    
# find the start time of the signal in the recorded data, function searches through complete data set, 
# if the magnitude is above 0.1 times the maximum magnitude int the recording it will test if its a false positive 
# inputs: Signal data, sample rate, number of samples
# outputs: signal start time 
def signal_start_time(sigData, sr, sampleN):
    
    # define the maximum and minimum error/threshold points to find signal start
    maxSig = max(sigData)
    error = maxSig*0.5
    maxError = maxSig*0.4
    maxError2 = maxSig*0.1

    minSig = min(sigData)
    error = minSig*0.5
    minError = minSig*0.4
    minError2 = minSig*0.1
    averageSig = average 

    x = 0
    noSigCounter = 0
    found = 0
    while x < sampleN:
        dataVAl = sigData[x]

        # count that there is no signal at the very start of the recording for 0.2 seconds
        if sigData[x] < maxError2 and sigData[x] > minError2:
            noSigCounter = noSigCounter + 1
            #print("counter at: ", x)
        else :
            noSigCounter = 0                                    #reset counter
            #print("reset at:", x)

        #begin searching for the signal start itme 
        if noSigCounter == 0.2*sr :
            #print("0.2 of no large signal at", x)
            while x < sampleN:
                
                if sigData[x] > maxError2 :

                    #print("more")
                    #print(sigData[x])                        
                    av = is_sig_outlier(x, sigData, sampleN)
                    if av > maxError2:
                        found = 1
                        break
                    else:
                        if x < (sampleN - 600):
                            x = x + 500

                if sigData[x] < minError2:
                    #print(sigData[x])
                    #print("less")
                    av = is_sig_outlier(x, sigData, sampleN)
                    if av > maxError2:
                        found = 1
                        break
                    else:
                        if x < (sampleN - 600):
                            x = x + 500
                x = x+1
        if found == 1:
            break

        x=x+1

    #print("number of samples:", x)
    if found == 1:
        T = sampleN/sr
        signalTime = (x/sampleN)*T
        #print("signal time:" ,signalTime)
        return signalTime
    else:
        return found
        
        
def angle_loc(dataNum):
        global signal_freq
	#setup the bandpass filter varibales
        wpassLow = signal_freq-750
        wpassHigh = signal_freq+750
        wstopLow = signal_freq-1500
        wstopHigh = signal_freq+1500

        #define passband and stopband
        wpass = [wpassLow, wpassHigh]
        wstop = [wstopLow, wstopHigh]
        gp = 3      # passband gain 
        gs = 40     # stopband gain 
	#intDataNum = int(dataNum)	
	#sound1 = 'src/py_bearing/py_bearing/audiofile/recGroup13/recNum5_ch1.wav' 
	#sound2 = 'src/py_bearing/py_bearing/audiofile/recGroup13/recNum5_ch2.wav' 
	
        sound1 = dataNum+'_ch1.wav'
        sound2 = dataNum+'_ch2.wav'
        #print(sound1)
        #print(sound2)
        sr1, sigData1 = wavfile.read(sound1)
        sr2, sigData2 = wavfile.read(sound2)
        #print("bandpass")
        sigFiltered1, N1 = bandPassFilter(sr1, sigData1, wpass, wstop, gp, gs)
        sigFiltered2, N2 = bandPassFilter(sr2, sigData2, wpass, wstop, gp, gs)

        #print("sig start time")
        Time1 = signal_start_time(sigFiltered1, sr1, N1)
        #nofilt1 = signal_start_time(sigData1, sr1, N1)
        Time2 = signal_start_time(sigFiltered2, sr2, N2)
        #nofilt2 = signal_start_time(sigData2, sr2, N2)

        print("time1 = ", Time1)
        #print("nofilt1 = ", nofilt1)
        print("time2 = ", Time2)
        #print("nofilt2 = ", nofilt2)

        if Time1 == 0 or Time2 == 0:
            print("no signal found")
            return 1000
        else:


            dTime = Time1 - Time2
            c = 1480
            d = 1.5

            sangle = (c*dTime)/d
            distAngle = abs(sangle)
            angle = np.arccos(distAngle)
            dangle = math.degrees(angle)
            #print(dangle)
            dangle = 90-dangle

            if dTime < 0:
                finalAngle = -dangle
            else:
                finalAngle = dangle

            print("final angle = ", finalAngle)

            passed_time = time.time() - start_time 
            time_taken = f'it took {passed_time}' 
            print(time_taken)
            return finalAngle
        #fft_graph(sr, aData.T[1])


def main(args=None):
    try:
        listener_publisher()
    except rospy.ROSInterruptException:
        pass



#    rclpy.init(args=args)
#
#    minimal_publisher = MinimalPublisher()
#
#    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
#    minimal_publisher.destroy_node()
#    rclpy.shutdown()
    
if __name__ == '__main__':
    main()






