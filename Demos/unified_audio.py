import numpy as np
import sounddevice as sd
import scipy
from transformers import pipeline

sd.query_devices()

#### DOA ####

# Start recording
n_samples = 15000
channels= 3
sample_rate = 44100
thresh = 13

stream = sd.InputStream(device=3, channels=channels, samplerate=sample_rate)

it = 0
calib = []
with stream:
    while True:
        samples, _ = stream.read(n_samples)
        offset = np.argmax(scipy.signal.correlate(samples[:,0], samples[:,1]))-n_samples
        if it < 5:
            calib.append(offset)
            it = it + 1
        else:
            if it == 5:
                zero = np.mean(calib)
                print(f"center offset: {zero}")
                it = it + 1
            if offset < zero-thresh:
                print("gauche")
            elif offset > zero+thresh:
                print("droite")
            else:
                print("indiscernable")
#### END DOA ####

#### KWS ####
classifier = pipeline("audio-classification", model="superb/wav2vec2-base-superb-ks")

for wav_data in record_audio():
    # pass the WAV data to your keyword spotter here
    label = classifier(wav_data, top_k=1)
    if label[0]["score"] > 0.99:
        print(label)
    else:
        print("No keyword detected", end = "\r")
    pass
#### END KWS ####