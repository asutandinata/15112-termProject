import aubio
import numpy as np

source=aubio.source('testMusic.wav')

print(source.channels, source.samplerate, source.duration)
framesCounted=0
for frame in source:
    framesCounted+=1
    print(framesCounted, frame.shape)
    