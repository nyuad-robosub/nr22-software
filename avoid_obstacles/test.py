import matplotlib.pyplot as plt
import numpy as np
import scipy.sparse as sp
from lazytheta import *
xlim = round(OCC_SIZE_X / 2)
ylim = round(OCC_SIZE_Y / 2)
zlim = round(OCC_SIZE_Z / 2)

# Generate some obstacles
# prepare some coordinates
u, v, w = np.indices(tuple(np.array(OCC_SIZE_ZIP) * 2))
# draw cuboids in the top left and bottom right corners, and a link between them
cube1 = (u > xlim - 2) & (u < xlim + 2) & (v > OCC_SIZE_Y - 4) & (w > zlim - 2) & (
        w < zlim + 2)
cube2 = (u > OCC_SIZE_X - 4) & (v > ylim - 2) & (v < ylim + 2) & (w > OCC_SIZE_Z - 4)
link = abs((u + 1) - (OCC_SIZE_Y - (v - ylim + 1))) + abs((OCC_SIZE_Y - (v - ylim + 1)) - w) \
       + abs(w - (u + 1)) <= 2
# link = (u < 0) & (v < 0) & (w < 0)
# combine the objects into a single boolean array
voxelarray = cube1 | cube2 | link
grid = []
# X: row, Y: column
row = []
col = []
data = []
for z in range(OCC_SIZE_Z):
    row.append([])
    col.append([])
    data.append([])

for x in range(len(voxelarray)):
    for y in range(len(voxelarray[x])):
        for z in range(len(voxelarray[x][y])):
            # X: row, Y: column
            if voxelarray[x][y][z]:
                row[z].append(x)
                col[z].append(y)
                data[z].append(OBSTACLE_THRESHOLD)

for z in range(OCC_SIZE_Z):
    grid.append(sp.coo_array((data[z], (row[z], col[z])), shape=(OCC_SIZE_X, OCC_SIZE_Y), dtype=np.uint8))

lt = LazyTheta()
tmp = set()
if lt.UpdateOccupancyGrid(grid=grid):
    tmp = lt.blockedXYZ

# set the expanded obstacles
obst = np.full((OCC_SIZE_X, OCC_SIZE_Y, OCC_SIZE_Z), False)
for (x, y, z) in tmp:
    if 0 <= x < OCC_SIZE_X:
        if 0 <= y < OCC_SIZE_Y:
            if 0 <= z < OCC_SIZE_Z:
                # X: row, Y: column
                obst[x][y][z] = True

xyzs = []
scttr_op = []
scttr_op_val = []
scttr_cl = []
scttr_cl_val = []
if lt.LineOfSight(Node((0, 0, 0)), Node((4, 4, 3))):
    print("LOS")

times = [0, 0, 0, 0]
import time
for i in range(100):
    print(i)
    t = time.time_ns() / 100000
    lt.UpdateOccupancyGrid(grid=grid)
    # USE_PTG_TYPE = i % 4
    lt.ComputePath((0, 0, 0), (8, 8, 8))
    times[0] += time.time_ns() / 100000 - t
print(times[0])

if (lt.ComputePath((0, 0, 0), (8, 7, 8))):
    s = lt.s_end
    xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
    print(s.xyz)
    while s.xyz != (0, 0, 0):
        s = s.parent
        xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
        print(s.xyz)

    scttr_op = [np.array(s.xyz) + np.array(OCC_SIZE_ZIP) for s in lt.openList]
    scttr_op_val = [s.fScore for s in lt.openList]
    scttr_cl = [np.array(s.xyz) + np.array(OCC_SIZE_ZIP) for s in lt.closed]
    scttr_cl_val = [s.fScore for s in lt.closed]

# set the colors of each object
colors = np.empty(voxelarray.shape, dtype=object)
colors[link] = 'red'
colors[cube1] = 'blue'
colors[cube2] = 'green'
colors[obst & (~voxelarray)] = '#FF993355'

# and plot everything
ax = plt.figure().add_subplot(projection='3d')
# ax.voxels(obst & (~voxelarray), facecolors=colors, edgecolor='k')
ax.voxels(voxelarray, facecolors=colors, edgecolor='k')
ax.plot(*zip(*xyzs), linewidth=2, zorder=1000)
# ax.scatter(*zip(*scttr_op), np.array(scttr_op_val))
# ax.scatter(*zip(*scttr_cl), np.array(scttr_cl_val))
# ax.scatter(*zip(*xyzs))
plt.show()