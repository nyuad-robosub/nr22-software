import random
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.sparse as sp
from lazytheta import *

xlim = round(OCC_SIZE_X / 2)
ylim = round(OCC_SIZE_Y / 2)
zlim = round(OCC_SIZE_Z / 2)
size = 6
start = tuple(- np.array(OCC_SIZE_ZIP))
end = tuple(np.array(OCC_SIZE_ZIP))
# filename = "log_{}.txt".format(random.randint(0, 1000))
# with open(filename, "w") as f:
#     f.write("Runtimes:\n")

random.seed(2930)
count = 0
while True:
    count += 1
    # Generate some obstacles
    obstSet = set()
    voxelarray = np.full((OCC_SIZE_X, OCC_SIZE_Y, OCC_SIZE_Z), False)
    obst = [(random.randint(0, OCC_SIZE_X - 1),
             random.randint(0, OCC_SIZE_Y - 1),
             random.randint(0, OCC_SIZE_Z - 1)) for _ in range(20)]
    for xyz in obst:
        if np.linalg.norm(np.array(xyz) - np.array(start) - np.array(OCC_SIZE_ZIP)) > (size + BOUND) * 1.75 and \
                np.linalg.norm(np.array(xyz) - np.array(end) - np.array(OCC_SIZE_ZIP)) > (size + BOUND) * 1.75:
            print("Vector3i", tuple(np.array(xyz) - np.array(OCC_SIZE_ZIP)), ",", sep='')
            for x in range(xyz[0] - size, xyz[0] + size + 1):
                for y in range(xyz[1] - size, xyz[1] + size + 1):
                    for z in range(xyz[2] - size, xyz[2] + size + 1):
                        if 0 <= x < OCC_SIZE_X and 0 <= y < OCC_SIZE_Y and 0 <= z < OCC_SIZE_Z:
                            # print("Vector3i", tuple(np.array([x, y, z]) - np.array(OCC_SIZE_ZIP)), ",")
                            voxelarray[x, y, z] = True
                            obstSet.add((x, y, z))
        # voxelarray[xyz[0]][xyz[1]][xyz[2]] = True

    # grid = []
    # # X: row, Y: column
    # row = []
    # col = []
    # data = []
    # for z in range(OCC_SIZE_Z):
    #     row.append([])
    #     col.append([])
    #     data.append([])

    # for x in range(len(voxelarray)):
    #     for y in range(len(voxelarray[x])):
    #         for z in range(len(voxelarray[x][y])):
    #             # X: row, Y: column
    #             if voxelarray[x][y][z]:
    #                 row[z].append(x)
    #                 col[z].append(y)
    #                 data[z].append(OBSTACLE_THRESHOLD)

    _start_time = time.time_ns()
    print("Pre")
    # for z in range(OCC_SIZE_Z):
    #    grid.append(sp.coo_array((data[z], (row[z], col[z])), shape=(OCC_SIZE_X, OCC_SIZE_Y), dtype=np.uint8))

    lt = LazyTheta()
    tmp = set()
    # if lt.UpdateOccupancyGrid(grid=grid):
    if lt.UpdateOccupancySet(occSet=obstSet):
        print(LazyTheta.times[3] / 1000000)
        tmp = lt.blockedXYZ

    # set the expanded obstacles
    obst = np.full((OCC_SIZE_X, OCC_SIZE_Y, OCC_SIZE_Z), False)
    for (x, y, z) in tmp:
        if 0 <= x < OCC_SIZE_X:
            if 0 <= y < OCC_SIZE_Y:
                if 0 <= z < OCC_SIZE_Z:
                    # X: row, Y: column
                    obst[x][y][z] = True
    print("Post")
    # Preview obstacles
    if False:
        colors = np.empty(voxelarray.shape, dtype=object)
        colors[voxelarray] = 'green'
        colors[obst & (~voxelarray)] = '#993355'
        ax = plt.figure().add_subplot(projection='3d')
        ax.voxels(obst & (~voxelarray), facecolors=colors, edgecolor='k')
        ax.voxels(voxelarray, facecolors=colors, edgecolor='k')
        ax.set_box_aspect(OCC_SIZE_ZIP)
        plt.show()

    xyzs = []
    scttr_op = []
    scttr_op_val = []
    scttr_cl = []
    scttr_cl_val = []
    # if lt.LineOfSight(Node(start), Node(end)):
    #     print("LOS")

    _times = [0, 0, 0, 0]
    for i in range(0):
        print(i)
        t = time.time_ns() / 100000
        lt.UpdateOccupancyGrid(grid=grid)
        # USE_PTG_TYPE = i % 4
        # lt.ComputePath((0, 0, 0), (8, 8, 8))
        _times[0] += time.time_ns() / 100000 - t
    # print(times[0])

    done = lt.ComputePath(start, end)
    duration = time.time_ns() - _start_time
    err = False
    if done:
        # with open(filename, "a") as f:
        #     f.write("{}\n".format(duration))
        print("Runtime:", duration / 1000000, "ms")
        print(" - GetVis:", LazyTheta.times[0] / 1000000, "ms", LazyTheta.times[0] / duration * 100, "%")
        print(" - LineOSight:", LazyTheta.times[1] / 1000000, "ms", LazyTheta.times[1] / duration * 100, "%")
        print("   + Regular 3-axis:", LazyTheta.times[5] / 1000000, "ms", LazyTheta.times[5] / LazyTheta.times[1] * 100,
              "%")
        print(" - PTGDist:", LazyTheta.times[2] / 1000000, "ms", LazyTheta.times[2] / duration * 100, "%")
        print(" - UpdateGrid:", LazyTheta.times[3] / 1000000, "ms", LazyTheta.times[3] / duration * 100, "%")
        print(" - Enumerate:", LazyTheta.times[4] / 1000000, "ms", LazyTheta.times[4] / duration * 100, "%")
        s = lt.s_end
        xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
        old_xyz = s.xyz
        print(s.xyz)
        while s.xyz != start:
            s = s.parent
            xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
            if not lt.LineOfSight(Node(s.xyz), Node(old_xyz)):
                print("ERR:", s.xyz, old_xyz)
                err = True
            old_xyz = s.xyz
            print(s.xyz)

    scttr_op = []
    scttr_op_val = []
    scttr_cl = []
    scttr_cl_val = []
    for xyz in list(lt.openXYZ):
        scttr_op.append(np.array(xyz) + np.array(OCC_SIZE_ZIP))
        scttr_op_val.append(lt.open[xyz].fScore)
    for xyz in list(lt.closedXYZ):
        scttr_cl.append(np.array(xyz) + np.array(OCC_SIZE_ZIP))
        scttr_cl_val.append(lt.closed[xyz].fScore)

    # err = False
    # for i in range(len(xyzs) - 1):
    #     if not lt.LineOfSight(Node(tuple(np.array(xyzs[i]) - np.array(OCC_SIZE_ZIP))),
    #                           Node(tuple(np.array(xyzs[i + 1]) - np.array(OCC_SIZE_ZIP)))):
    #         err = True

    if duration / 1000000000 > 8:  # err anderr or not done
        # set the colors of each object
        colors = np.empty(voxelarray.shape, dtype=object)
        colors[voxelarray] = 'green'
        colors[obst & (~voxelarray)] = '#FF993355'

        # and plot everything
        ax = plt.figure().add_subplot(projection='3d')
        # ax.voxels(obst & (~voxelarray), facecolors=colors, edgecolor='k')
        ax.voxels(voxelarray, facecolors=colors, edgecolor='k')
        # ax.plot(*zip(*xyzs), linewidth=2, zorder=1000)
        # ax.scatter(*zip(*scttr_op), np.array(scttr_op_val))
        # ax.scatter(*zip(*scttr_cl), np.array(scttr_cl_val))
        # ax.scatter(*zip(*xyzs), s=10, c='#882222FF')
        # for i in range(len(xyzs) - 1):
        #     ax.plot([xyzs[i][0], xyzs[i + 1][0]],
        #             [xyzs[i][1], xyzs[i + 1][1]],
        #             [xyzs[i][2], xyzs[i + 1][2]], linewidth=2)
        for i in range(len(xyzs) - 1):
            for t in np.arange(0.0, 1.0, 0.2):
                ax.scatter(xyzs[i][0] * t + xyzs[i + 1][0] * (1 - t),
                           xyzs[i][1] * t + xyzs[i + 1][1] * (1 - t),
                           xyzs[i][2] * t + xyzs[i + 1][2] * (1 - t))
        # if err:
        #     for i in range(len(xyzs) - 1):
        #         for t in np.arange(0.0, 1.0, 0.2):
        #             ax.scatter(xyzs[i][0] * t + xyzs[i + 1][0] * (1 - t),
        #                         xyzs[i][1] * t + xyzs[i + 1][1] * (1 - t),
        #                         xyzs[i][2] * t + xyzs[i + 1][2] * (1 - t))
        # ax.scatter(*zip(*scttr_op), np.array(scttr_op_val))
        # ax.scatter(*zip(*scttr_cl), np.array(scttr_cl_val))
        # ax.plot([xyzs[i][0], xyzs[i + 1][0]],
        #         [xyzs[i][1], xyzs[i + 1][1]],
        #         [xyzs[i][2], xyzs[i + 1][2]], linewidth=2)
        ax.set_box_aspect(OCC_SIZE_ZIP)
        plt.show()
