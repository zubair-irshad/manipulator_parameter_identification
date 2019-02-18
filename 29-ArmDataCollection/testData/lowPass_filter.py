#!/usr/bin/env python

from __future__ import division
import numpy as np
import matplotlib.pyplot as pl
import scipy

from scipy.signal import butter, lfilter, freqz
#import matplotlib.pyplot as pl


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


# Filter requirements.
order = 2
fs = 96.2698       # sample rate, Hz
cutoff = 3.0#2.2322  # desired cutoff frequency of the filter, Hz
cutoff_ddq = 2.0




with open('dataTimeStamp.txt') as f:
    timeStamp = f.read().split()
timeStamp = np.array(timeStamp[:])
timeStamp = map(float, timeStamp)
t = timeStamp[:]
print len(t)
N = max(t)



with open('dataDotQ_NF.txt') as f:
    dataDotQ_NF = f.read().split()
dataDotQ_NF = np.array(dataDotQ_NF[:])
dataDotQ_NF = map(float, dataDotQ_NF)
dataDotQ_NF = np.reshape(dataDotQ_NF,(-1, 7))
print len(dataDotQ_NF)


dataDotQ = []
for joint in range(7):
    dataDotQ_f = butter_lowpass_filter(dataDotQ_NF[:,joint], cutoff, fs, order)
    dataDotQ.append(dataDotQ_f)
dataDotQ = np.array(dataDotQ)
dataDotQ = np.transpose(dataDotQ)

dataDDotQ = []
for joint in range(7):
    dataDDotQ_f = np.diff(dataDotQ[:,joint])/np.diff(t)
    dataDDotQ.append(dataDDotQ_f)
dataDDotQ = np.array(dataDDotQ)
dataDDotQ = np.transpose(dataDDotQ)
print len(dataDDotQ)

# dataDDotQ_NF = []
# for joint in range(7):
#     dataDDotQ_NF_f = np.diff(dataDotQ_NF[:,joint])/np.diff(t)
#     dataDDotQ_NF.append(dataDDotQ_NF_f)
# dataDDotQ_NF = np.array(dataDDotQ_NF)
# dataDDotQ_NF = np.transpose(dataDDotQ_NF)
# print len(dataDDotQ_NF)

dq = dataDotQ
np.savetxt('dataDotQ.txt', dq, fmt='%4e')

ddq = dataDDotQ
np.savetxt('dataDDotQ.txt', ddq, fmt='%4e')

for JOINT in range(1,8):

    pl.figure(JOINT)
    pl.clf()
    pl.plot(t, dataDotQ_NF[:,JOINT-1], 'g-', label='dataDotQ_NF')
    pl.plot(t, dataDotQ[:,JOINT-1], 'r-', label='dataDotQ')
    # pl.plot(t[1:], dataDDotQ_NF[:,JOINT-1], 'c-', label='ddq_NF')
    pl.plot(t[1:], dataDDotQ[:,JOINT-1], 'b-', label='ddq')
    pl.xlabel('Time [sec]')
    pl.grid()
    pl.legend()
pl.show()