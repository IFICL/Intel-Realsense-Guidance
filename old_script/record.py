import time
t0 = time.time()
import sounddevice as sd
import sys
import soundfile as sf
import numpy as np
from scipy.io.wavfile import write

fs = 44100  # Sample rate
# sd.default.samplerate = fs
# sd.default.channels = 2
sd.default.device = [12, 8]
# sd.default.device = 'digital output'
# python -m sounddevice

if __name__ == "__main__":
    seconds = 10  # Duration of recording
    print('Start recording')
    t1 = time.time()
    myrecording = sd.rec(int(seconds * fs), samplerate=fs, channels=2, dtype='int16')
    sd.wait()  # Wait until recording is finished
    t2 = time.time()
    write('output.wav', fs, myrecording)  # Save as WAV file
    print('Wait time is', t1 - t0)
    print('Record time is', t2 - t1)
    # print('Sleep for 3 seconds')
    # time.sleep(3)   
    # sd.playrec(myrecording, fs, channels=2, blocking=True)

    # data, samplerate = sf.read('output.wav')
    # print(data)
    # sd.play(data, samplerate, blocking=True)






























# import pyaudio
# import wave

# chunk = 1024  # Record in chunks of 1024 samples
# sample_format = pyaudio.paInt16  # 16 bits per sample
# channels = 2
# fs = 44100  # Record at 44100 samples per second
# seconds = 3
# filename = "output.wav"

# p = pyaudio.PyAudio()  # Create an interface to PortAudio

# print('Recording')

# stream = p.open(format=sample_format,
#                 channels=channels,
#                 rate=fs,
#                 frames_per_buffer=chunk,
#                 input=True)

# frames = []  # Initialize array to store frames

# # Store data in chunks for 3 seconds
# for i in range(0, int(fs / chunk * seconds)):
#     data = stream.read(chunk)
#     frames.append(data)

# # Stop and close the stream
# stream.stop_stream()
# stream.close()
# # Terminate the PortAudio interface
# p.terminate()

# print('Finished recording')

# # Save the recorded data as a WAV file
# wf = wave.open(filename, 'wb')
# wf.setnchannels(channels)
# wf.setsampwidth(p.get_sample_size(sample_format))
# wf.setframerate(fs)
# wf.writeframes(b''.join(frames))
# wf.close()
