from matplotlib import projections
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--filename', '-f', type=str, default='./log/gps_vel_static_30s')
args = parser.parse_args()

time = []
north = []
east = []


with open(args.filename) as file:
    print('args=', str(args.filename))
    content = file.readlines()
    for data in content:
        data = data.split(',')
        time.append(int(data[0].split(' ')[3]))
        north.append(float(data[1].split(' ')[3]) * 1e-3)
        east.append(float(data[2].split(' ')[3]) * 1e-3)
        
        # print(data[0].split(' ')[3])
        # print(data[1].split(' ')[3])
        # print(data[2].split(' ')[3])
        

time = np.array(time)
north = np.array(north)
east = np.array(east)

fig = plt.figure()


# ax = plt.gca(projection='3d')
# ax.plot(north, east, time, linewidth = 3)
# ax.plot(north, east, zs=np.min(time))


ax = plt.gca()
plt.plot(east, north, color = 'green', linewidth = 2)

# lat_major_locator = plt.MultipleLocator(0.0002)
# ax.yaxis.set_major_locator(lat_major_locator)

# plt.ylim(37.7726, 37.7744)

plt.xlabel('East')
plt.ylabel('North')
# plt.xticks([])
# plt.yticks([])
plt.show()

