#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import scipy
import matplotlib.pyplot as plt
from transformers import pipeline
from scipy.io.wavfile import write

classifier = pipeline("audio-classification", model="superb/wav2vec2-base-superb-ks") # create kws spotter model

# constants for audio input
DEVICE = 25 #check this before in a terminal by executing "python3 search_auido_devices.py"
CHANNELS = 3
RATE = 16000

if __name__ == '__main__':
    # init the ros nodes 
    rospy.init_node('audio_process', anonymous=True)

    pub_audio = rospy.Publisher('audio_info_topic', String, queue_size=10)

    it = 0
    n_samples = 8000 # 0.5s - will also put a delay of 0.5s (2hz)
    thresh = 3 #manually tuned depends on the sampling rate !
    calib = []

    while(True):
        recording = sd.rec(frames=n_samples, device=DEVICE, channels=CHANNELS, samplerate=RATE)
        sd.wait()
        offset = np.argmax(scipy.signal.correlate(recording[:,0], recording[:,2]))-n_samples
        if it < 5: # calibration for the first 2s 
            if it == 0:
                print("Calibration for DOA be quiet")
            calib.append(offset)
            it = it + 1
        if it == 5:
            print("Calibration done")
            zero = np.mean(calib)
            print(f"center offset: {zero}")
            it = it + 1
        if it > 5:
            ### KWS ###
            write('temp.wav', 16000, 3*recording[:,1])  # Save as WAV file only the sennhesier
            label = classifier("temp.wav", top_k=1)
            if label[0]["score"] > 0.95:
                keyword = label[0]["label"]
            else:
                keyword = "No keyword detected"
            ### DOA ###
            if offset < zero-thresh:
                doa = "left"
            elif offset > zero+thresh:
                doa = "right"
            else:
                doa = "center or unidentified"
            
            pub_audio.publish(f"{keyword}#{doa}")


