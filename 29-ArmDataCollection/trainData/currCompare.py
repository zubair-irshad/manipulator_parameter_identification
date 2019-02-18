#!/usr/bin/env python

from __future__ import division
import numpy as np
import matplotlib.pyplot as pl




# Sample some input points and noisy versions of the function evaluated at
# these points. 

# Load training set
#qTrain = scipy.io.loadmat("dataSet1/trainingData/dataQ.txt")
#XqTrain = np.array(qTrain["dataSet1/trainingData/dataQ"][:N, :7])

#N = 900;
#wf = 0.558048373585;
#amp = 0.8;

# T = 7 * N
# S = 2 * N



with open('dataTimeStamp.txt') as f:
    timeStamp = f.read().split()
timeStamp = np.array(timeStamp[:])
timeStamp = map(float, timeStamp)
print len(timeStamp)
N = len(timeStamp)


with open('dataQ.txt') as f:
    dataQ = f.read().split()
dataQ = np.array(dataQ[:])
dataQ = map(float, dataQ)
dataQ = np.reshape(dataQ,(-1, 7))
#dataQ = np.reshape(dataQ,(-1, 7))
#dataQ = dataQ[:N, 0]
print len(dataQ)


with open('dataDotQ_NF.txt') as f:
    dataDotQ = f.read().split()
dataDotQ = np.array(dataDotQ[:])
dataDotQ = map(float, dataDotQ)
dataDotQ = np.reshape(dataDotQ,(-1, 7))


# with open('/home/munzir/Documents/Software/experiments/teleop/build/dataError.txt') as f:
#     error = f.read().split()
# error = np.array(error[:N])
# error = map(float, error)



with open('dataCur.txt') as f:
    meas_curr = f.read().split()
meas_curr = np.array(meas_curr[:])
meas_curr = map(float, meas_curr)
meas_curr = np.reshape(meas_curr,(-1, 7))
print len(meas_curr)

with open('dataQref.txt') as f:
    q_ref = f.read().split()
q_ref = np.array(q_ref[:])
q_ref = map(float, q_ref)
q_ref = np.reshape(q_ref,(-1, 7))
print len(q_ref)

with open('dataDQref.txt') as f:
    dq_ref = f.read().split()
dq_ref = np.array(dq_ref[:])
dq_ref = map(float, dq_ref)
dq_ref = np.reshape(dq_ref,(-1, 7))
print len(dq_ref)






# q_ref = []
# for i in range(len(timeStamp)):
# 	qref0 = amp*np.sin(wf*timeStamp[i])
# 	q_ref = np.append(q_ref, qref0)
# #print len(q_ref)

# dq_ref = []
# for i in range(len(timeStamp)):
# 	dqref0 = amp*wf*np.cos(wf*timeStamp[i])
# 	dq_ref = np.append(dq_ref, dqref0)


# curr = []
# for i in range(len(timeStamp)):
# 	curr0 = 4*np.sin(wf*timeStamp[i])
# 	curr = np.append(curr, curr0)


N = max(timeStamp)

#ttt = 'LS = {} , sigmaSqF = {}, sigmaSqN = {}, train = {}, test = {}'.format(lS, sigmaSqF, sigmaSqN, N, n)
#ttt = 'Joint 1, LS = {}'.format(lS)


for joint in range(1, 8):

	trial = 001
	#joint = 7

	title1 = '#{} Joint {} position trial'.format(trial, joint)
	titleS1 = 'plots/joint{}/'.format(joint)+ title1
	#print title

	title2 = '#{} Joint {} velocity trial'.format(trial, joint)
	titleS2 = 'plots/joint{}/'.format(joint)+ title2

	# title3 = 'Joint 7 current trial {}'.format(trial)
	# titleS3 = '/home/munzir/Documents/Software/Victor/July_30/plots/joint7/pose3/'+ title3

	title3 = '#{} Joint {} current_measured trial'.format(trial, joint)
	titleS3 = 'plots/joint{}/'.format(joint)+ title3


	
	ymin = min(meas_curr[:,joint-1])
	ymax = max(meas_curr[:,joint-1])
	print 'ymin ', min(meas_curr[:,joint-1])
	print 'ymax ', max(meas_curr[:,joint-1])


	# PLOTS:
	pl.figure(3*joint+1)
	pl.clf()
	pl.plot(timeStamp, q_ref[:,joint-1], 'r-',ms = 10)
	pl.plot(timeStamp, dataQ[:,joint-1], 'b-', ms = 10)#, label="Test values")
	pl.title(title1)
	pl.gca().legend(('q_ref','q_measured'), loc=0, labelspacing=0.5)
	pl.savefig(titleS1, bbox_inches='tight')
	#pl.axis([0,n,ymin,ymax])
	#pl.show()

	pl.figure(3*joint+2)
	pl.clf()
	pl.plot(timeStamp, dq_ref[:,joint-1], 'r-',ms = 10)
	pl.plot(timeStamp, dataDotQ[:,joint-1], 'g-', ms = 10)#, label="Test values")
	pl.title(title2)
	pl.gca().legend(('dq_ref','dq_measured'), loc=0, labelspacing=0.5)
	pl.savefig(titleS2, bbox_inches='tight')
	# pl.axis([0,n,ymin,ymax])


	pl.figure(3*joint+3)
	pl.clf()
	pl.plot(timeStamp, meas_curr[:,joint-1], 'g-',ms = 10)
	pl.title(title3)
	pl.gca().legend(('error'), loc=0, labelspacing=0.5)
	pl.savefig(titleS3, bbox_inches='tight')
	# pl.axis([0,n,ymin,ymax])
	#pl.show()

pl.show()