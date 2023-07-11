#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import scipy
import matplotlib.pyplot as plt
from transformers import pipeline
from scipy.io.wavfile import write

#import pyaudio
import pickle 
import wave
import numpy as np
import torch
import torchaudio
import torch.nn.functional as F
from transformers import AutoModelForAudioClassification, AutoConfig, AutoFeatureExtractor


#classifier = pipeline("audio-classification", model="superb/wav2vec2-base-superb-ks") # create kws spotter model

# constants for audio input
DEVICE = 36 #check this before in a terminal by executing "python3 search_auido_devices.py"
CHANNELS = 3
RATE = 16000

def speech_file_to_array_fn(path, sampling_rate):
    speech_array, _sampling_rate = torchaudio.load(path)
    resampler = torchaudio.transforms.Resample(_sampling_rate)
    speech = resampler(speech_array).squeeze().numpy()
    return speech


def predict(path, sampling_rate):
    speech = speech_file_to_array_fn(path, sampling_rate)
    features = feature_extractor(speech, sampling_rate=sampling_rate, return_tensors="pt", padding=True)

    input_values = features.input_values.to(device)
    # attention_mask = features.attention_mask.to(device)

    with torch.no_grad():
        logits = model(input_values).logits

    scores = F.softmax(logits, dim=1).detach().cpu().numpy()[0]
    outputs = scores
    # outputs = [{"Emotion": config.id2label[i], "Score": f"{round(score * 100, 3):.1f}%"} for i, score in enumerate(scores)]
    return outputs

if __name__ == '__main__':
    # init the ros nodes 
    rospy.init_node('audio_process', anonymous=True)
    pub_audio = rospy.Publisher('audio_info_topic', String, queue_size=10)

    # create everything related to the custom kws model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    feature_extractor_checkpoint = "facebook/wav2vec2-base"
    audio_classification_checkpoint = "kws_model"
    feature_extractor = AutoFeatureExtractor.from_pretrained(feature_extractor_checkpoint)
    config = AutoConfig.from_pretrained(feature_extractor_checkpoint)
    sampling_rate = feature_extractor.sampling_rate
    model = AutoModelForAudioClassification.from_pretrained(audio_classification_checkpoint).to(device)
    with open("id2label.pkl", 'rb') as f:
        id2label = pickle.load(f)


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
            write('temp.wav', 16000, recording[:,1])  # Save as WAV file only the sennhesier
            label = predict("temp.wav", 16000)
            max = np.argmax(label)
           # print(f"most confident keyword = {id2label[str(max)]}, with a confiden of {label[max]}")
           # keyword = "testing"
            
            if label[max] > 0.95:
                keyword = id2label[str(max)]
            else:
                keyword = "no keyword detected"
            
            ### DOA ###
            if offset < zero-thresh:
                doa = "left"
            elif offset > zero+thresh:
                doa = "right"
            else:
                doa = "center or unidentified"
            
            pub_audio.publish(f"{keyword}#{doa}")


