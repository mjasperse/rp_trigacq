import pylab as pyl
import numpy as np

data = np.fromfile("data.dat",dtype='<f4',count=-1).reshape((-1,3))
pyl.plot(data[:,0],data[:,1],'x-')
pyl.plot(data[:,0],data[:,2],'x-')
pyl.show()