import random
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.sparse as sp
from lazytheta import *

xlim = round(OCC_SIZE_X / 2)
ylim = round(OCC_SIZE_Y / 2)
zlim = round(OCC_SIZE_Z / 2)
size = 8
# filename = "log_{}.txt".format(random.randint(0, 1000))
# with open(filename, "w") as f:
#     f.write("Runtimes:\n")

random.seed(2930)
count = 0
while True:
    count += 1
    # Generate some obstacles
    voxelarray = np.full((OCC_SIZE_X, OCC_SIZE_Y, OCC_SIZE_Z), False)
    # Gate creation
    for z in range(0, round(1 / OCC_UNIT_Z)):
        voxelarray[round(4 / OCC_UNIT_X)][round(1 / OCC_UNIT_Y)][z] = True
        voxelarray[round(4 / OCC_UNIT_X)][round(3 / OCC_UNIT_Y)][z] = True
    for y in range(round(1 / OCC_UNIT_Y), round(3 / OCC_UNIT_Y) + 1):
        voxelarray[round(4 / OCC_UNIT_X)][y][round(1 / OCC_UNIT_Z)] = True
    # Marker creation
    X_CENTER, Y_CENTER = (14, 2)
    RADIUS = 0.5
    for xy_ in [(x0, y0) for x0 in range(round((X_CENTER - RADIUS) / OCC_UNIT_X),
                                         round((X_CENTER + RADIUS) / OCC_UNIT_X + 1))
                for y0 in range(round((Y_CENTER - RADIUS) / OCC_UNIT_Y),
                                round((Y_CENTER + RADIUS) / OCC_UNIT_Y + 1))]:
        # print(xy_)
        if np.linalg.norm(np.array((X_CENTER / OCC_UNIT_X, Y_CENTER / OCC_UNIT_Y))
                          - np.array(xy_)) < RADIUS / OCC_UNIT:
            for z in range(0, OCC_SIZE_Z, 6):
                voxelarray[xy_[0]][xy_[1]][z] = True
    # Waypoints (can be dynamically defined later)
    start = (- OCC_SIZE_X // 2, 0,
             # random.randint(- OCC_SIZE_Y // 2, OCC_SIZE_Y // 2),
             random.randint(- OCC_SIZE_Z // 2, 0))
    end = (- OCC_SIZE_X // 2, 0,
           # random.randint(- OCC_SIZE_Y // 2, OCC_SIZE_Y // 2),
           random.randint(- OCC_SIZE_Z / 2, 0))
    process = [start,
               (-20, 4, -10),  # Past gate
               (75, 4, -5),  # Left side of marker
               (75, -4, -5),  # Left side of marker
               (-20, -4, -10),  # Before gate
               end]

    occSet = set()

    for x in range(len(voxelarray)):
        for y in range(len(voxelarray[x])):
            for z in range(len(voxelarray[x][y])):
                # X: row, Y: column
                if voxelarray[x][y][z]:
                    occSet.add((x, y, z))

    _start_time = time.time_ns()
    print("Pre")

    lt = LazyTheta()
    tmp = set()
    if lt.UpdateOccupancySet(occSet=occSet):
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
    # C++ generated path
    xyzs_ = [[(-75, -14, -13), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-75, 15, -12)
    ], [(-75, -5, -8), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-49, -5, -9), (-75, -8, -1)
    ], [(-75, 1, -9), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-42, 0, -9), (-43, 0, -8), (-75, 7, -2)
    ], [(-75, -1, -5), (-30, 3, -10), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-45, -5, -9), (-75, -14, -8)
    ], [(-75, 6, 0), (-35, 4, -10), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-47, -4, -9), (-75, -4, -7)
    ], [(-75, -12, -11), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-75, 9, -12)
    ], [(-75, -10, -2), (-34, 0, -10), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-40, -5, -9), (-75, -17, -2)
    ], [(-75, -5, -7), (-21, 4, -10), (-20, 4, -10), (-20, 4, -10), (68, 10, -5), (72, 9, -5), (73, 8, -5), (74, 7, -5), (75, 4, -5), (75, 4, -5), (75, -4, -5), (75, -4, -5), (74, -6, -5), (71, -9, -5), (-20, -4, -10), (-20, -4, -10), (-39, -5, -11), (-75, -18, -14)
    ]]

    # Preview obstacles
    if True:
        colors = np.empty(voxelarray.shape, dtype=object)
        colors[voxelarray] = 'green'
        # colors[obst & (~voxelarray)] = '#993355'
        ax = plt.figure().add_subplot(projection='3d')
        # ax.voxels(obst & (~voxelarray), facecolors=colors, edgecolor='k')
        ax.voxels(voxelarray, facecolors=colors, edgecolor='k')
        ax.set_box_aspect(OCC_SIZE_ZIP)

        for i in range(len(xyzs_[count - 1]) - 1):
            if not lt.LineOfSight(Node(xyzs_[count - 1][i - 1]), Node(xyzs_[count - 1][i])):
                print("ERR:", xyzs_[count - 1][i], xyzs_[count - 1][i + 1])
                err = True
            for t in np.arange(0.0, 1.0, 0.2):
                ax.scatter(OCC_SIZE_ZIP[0] + xyzs_[count - 1][i][0] * t + xyzs_[count - 1][i + 1][0] * (1 - t),
                           OCC_SIZE_ZIP[1] + xyzs_[count - 1][i][1] * t + xyzs_[count - 1][i + 1][1] * (1 - t),
                           OCC_SIZE_ZIP[2] + xyzs_[count - 1][i][2] * t + xyzs_[count - 1][i + 1][2] * (1 - t))
        plt.show()

    if count == len(xyzs_):
        break
    continue
    xyzs = []
    scttr_op = []
    scttr_op_val = []
    scttr_cl = []
    scttr_cl_val = []

    for i in range(len(process) - 1):
        done = lt.ComputePath(process[i], process[i + 1])
        duration = time.time_ns() - _start_time
        err = False
        if done:
            # with open(filename, "a") as f:
            #     f.write("{}\n".format(duration))
            print("Runtime:", duration / 1000000, "ms")
            print(" - GetVis:", LazyTheta.times[0] / 1000000, "ms", LazyTheta.times[0] / duration * 100, "%")
            print(" - LineOSight:", LazyTheta.times[1] / 1000000, "ms", LazyTheta.times[1] / duration * 100, "%")
            print("   + Regular 3-axis:", LazyTheta.times[5] / 1000000, "ms",
                  LazyTheta.times[5] / LazyTheta.times[1] * 100,
                  "%")
            print(" - PTGDist:", LazyTheta.times[2] / 1000000, "ms", LazyTheta.times[2] / duration * 100, "%")
            print(" - UpdateGrid:", LazyTheta.times[3] / 1000000, "ms", LazyTheta.times[3] / duration * 100, "%")
            print(" - Enumerate:", LazyTheta.times[4] / 1000000, "ms", LazyTheta.times[4] / duration * 100, "%")
            s = lt.s_end
            proto_xyzs = []
            proto_xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
            old_xyz = s.xyz
            print(s.xyz)
            while s.xyz != process[i]:
                s = s.parent
                proto_xyzs.append([s.xyz[i] + OCC_SIZE_ZIP[i] for i in range(3)])
                # if not lt.LineOfSight(Node(s.xyz), Node(old_xyz)):
                #     print("ERR:", s.xyz, old_xyz)
                #     err = True
                old_xyz = s.xyz
                print(s.xyz)
            proto_xyzs.reverse()
            xyzs += proto_xyzs

            # colors = np.empty(voxelarray.shape, dtype=object)
            # colors[voxelarray] = 'green'
            # colors[obst & (~voxelarray)] = '#FF993355'
            #
            # # and plot everything
            # ax = plt.figure().add_subplot(projection='3d')
            # # ax.voxels(obst & (~voxelarray), facecolors=colors, edgecolor='k')
            # ax.voxels(voxelarray, facecolors=colors, edgecolor='k')
            # # for i in range(len(xyzs) - 1):
            # #     ax.plot([xyzs[i][0], xyzs[i + 1][0]],
            # #             [xyzs[i][1], xyzs[i + 1][1]],
            # #             [xyzs[i][2], xyzs[i + 1][2]], linewidth=2)
            # for i in range(len(xyzs) - 1):
            #     for t in np.arange(0.0, 1.0, 0.2):
            #         ax.scatter(xyzs[i][0] * t + xyzs[i + 1][0] * (1 - t),
            #                    xyzs[i][1] * t + xyzs[i + 1][1] * (1 - t),
            #                    xyzs[i][2] * t + xyzs[i + 1][2] * (1 - t))
            # ax.set_box_aspect(OCC_SIZE_ZIP)
            # plt.show()

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

    if duration / 1000000000 > 0:  # err anderr or not done
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
