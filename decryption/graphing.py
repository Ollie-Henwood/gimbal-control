import numpy as np
from matplotlib import pyplot as plt
import csv

try:
    filenum = int(input('Input csv file number to graph: '))
except:
    print('Please enter an integer.')

with open('output' + str(filenum) + '.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))

values = np.asarray(data[1:-1])
values = values.astype(np.float32)

time_offset = values[1][0] #take off 
time = (values[:,0]-time_offset) / 1000000 #microseconds to seconds

plt.plot(time, values[:,1], label='PID Error x')
plt.plot(time, values[:,2], label='P x')
plt.plot(time, values[:,3], label='I x')
#plt.plot(time, values[:,4], label='D x')
#plt.plot(time, values[:,5], label='PID Error y')
#plt.plot(time, values[:,6], label='P y')
#plt.plot(time, values[:,7], label='I y')
#plt.plot(time, values[:,8], label='D y')
plt.plot(time, values[:,10], label='Mode')
plt.xlabel('Time (s)')
plt.ylabel('(Degrees)')
plt.legend()
plt.show()