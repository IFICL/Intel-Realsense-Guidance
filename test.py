import time
import argparse
import numpy as np
import sys
import os
from threading import Thread
import multiprocessing
import glob

import sounddevice as sd
import soundfile as sf
import pyrealsense2 as rs
import cv2

import pdb
import geocoder

# def main():
#     pdb.set_trace()
#     sd.default.device = [7, 4]
#     print(sd.query_devices())
#     filename = 'test.wav'
#     fs = 48000
#     myrecording = sd.rec(int(10 * fs), samplerate=fs, channels=2, dtype='float64')
#     sd.wait()
#     sf.write(filename, myrecording, fs)

def main():
    g = geocoder.ip('me')
    pdb.set_trace()
    print(g.latlng)

if __name__ == '__main__':
    main()