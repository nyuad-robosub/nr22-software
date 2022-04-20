import math
import numpy as np
import scipy.sparse as sp
import time

# Using SortedKeyList with key-get lambda
# https://stackoverflow.com/questions/27672494/how-to-use-bisect-insort-left-with-a-key
from sortedcontainers.sortedlist import SortedKeyList

# !!! Important notes !!!
# Coordinates for the obstacle grid is different from nodes
# Grid from 0 to OCC_SIZE
# Node from -OCC_SIZE / 2 to OCC_SIZE / 2

# Define some constants
# Size of occupancy grid (let ROV start at center)
OCC_SIZE_X = 24
OCC_SIZE_Y = 24
OCC_SIZE_Z = 10
OCC_SIZE_ZIP = [round(OCC_SIZE_X / 2), round(OCC_SIZE_Y / 2), round(OCC_SIZE_Z / 2)]
# Unit dimensions of occupancy grid (meters)
OCC_UNIT_X = OCC_UNIT_Y = OCC_UNIT_Z = OCC_UNIT = 0.1
# Determining characteristics of obstacle definition
OBSTACLE_THRESHOLD = 255
# Define boundaries for pool surfaces
MAX_HEIGHT = 50
MAX_DEPTH = -50
# Define boundaries for obstacles (truncated box)
AVOIDANCE_RADIUS = 0.2 # meters
BOUND_X = BOUND_Y = BOUND_Z = BOUND = round(AVOIDANCE_RADIUS / OCC_UNIT)
# Get costs of neighbors
NGBR_COST = (1, 1.41421, 1.73205)

# For analytics
USE_PTG_TYPE = 0

# Different schemes to find Pythagoras distance
# Integer parameters
def GetPTGDistance(xyz_start, xyz_end):
    if USE_PTG_TYPE == 0:
        start_time = time.time_ns()
        val = math.sqrt((xyz_start[0] - xyz_end[0]) ** 2 +
                         (xyz_start[1] - xyz_end[1]) ** 2 +
                         (xyz_start[2] - xyz_end[2]) ** 2)
        LazyTheta.times[2] += time.time_ns() - start_time
        return val
    elif USE_PTG_TYPE == 1:
        return np.linalg.norm(np.array(xyz_start) - np.array(xyz_end))
    elif USE_PTG_TYPE == 2:
        xyz = tuple(np.sort(np.array(xyz_start) - np.array(xyz_end)))
        try:
            return GetPTGDistance.XYZ[xyz]
        except AttributeError:
            GetPTGDistance.XYZ = {}
        except KeyError:
            pass
        val = math.sqrt((xyz_start[0] - xyz_end[0]) ** 2 +
                         (xyz_start[1] - xyz_end[1]) ** 2 +
                         (xyz_start[2] - xyz_end[2]) ** 2)
        GetPTGDistance.XYZ[xyz] = val
        return val
    else:
        xyz = tuple(np.sort(np.array(xyz_start) - np.array(xyz_end)))
        try:
            return GetPTGDistance.XYZ[xyz]
        except AttributeError:
            GetPTGDistance.XYZ = {}
        except KeyError:
            pass
        val = np.linalg.norm(np.array(xyz_start) - np.array(xyz_end))
        GetPTGDistance.XYZ[xyz] = val
        return val


# Node/vertex class
# Position coordinates (x, y, z): integers
# Scores (gScore, fScore): float/float64
class Node:
    def __init__(self, xyz=(0, 0, 0), gScore=math.inf, goal=None, parent=None):
        self.xyz = xyz
        self.gScore = gScore
        self.hScore = 0
        self.fScore = gScore
        self.parent = parent

        if goal is not None:
            self.UpdateHScore(goal)

    def UpdateGScore(self, gScore=math.inf):
        self.gScore = gScore
        self.fScore = gScore + self.hScore

    # Take advantage of integers for faster calculations
    def UpdateHScore(self, goal):
        self.hScore = GetPTGDistance(self.xyz, goal.xyz)


# LazyTheta class
# For newing instances
class LazyTheta:
    times = [0, 0, 0, 0, 0]
    # ---
    # Initialization & updates area
    # ---
    def __init__(self, zMax=OCC_SIZE_Z, zMin=0, xyzStart=(0, 0, 0)):
        # To-explore stack
        self.open = {}
        # To-explore stack as an fScore sorted list
        self.openList = SortedKeyList(key=lambda r: r.fScore)
        # To-explore xyz as unordered set (for quick neighbor lookup)
        self.openXYZ = set()
        # Explored stack
        self.closed = {}
        # Explored xyz as unordered set (for quick neighbor lookup)
        self.closedXYZ = set()
        # Occupancy grid
        self.grid = []
        for z in range(OCC_SIZE_Z):
            # X: row, Y: column
            self.grid.append(sp.lil_array((OCC_SIZE_X, OCC_SIZE_Y), dtype=np.uint8))

        # Blocked grid in form of set of tuples (most efficient)
        self.blockedXYZ = set()
        # Set of registered central obstacle cells
        self.obstaclesXYZ = set()
        # Get water surface z
        self.zMax = zMax
        # Get water depth z
        self.zMin = zMin

        # Placeholder start node
        self.s_start = None
        self.s_end = None

        LazyTheta.times = [0, 0, 0, 0, 0, 0]

    # Update occupancy grid, with updateStart and updateBox tuples of 3D ints
    def UpdateOccupancyGrid(self, grid=None, updateStart=None, updateBox=None):
        start_time = time.time_ns()
        # Update occupancy grid, with updateStart and updateBox tuples of 3D ints
        if grid is None:
            return False
        for z in range(len(grid)):
            self.grid[z] = sp.lil_array(grid[z])

        # If updateStart is None: function called to initialize obstacles
        if updateStart is None:
            for z in range(self.zMin, self.zMax):
                # X: row, Y: column
                X, Y, V = sp.find(grid[z])
                for i in range(len(V)):
                    if V[i] >= OBSTACLE_THRESHOLD:
                        # Add obstacle if not already added
                        if (X[i], Y[i], z) not in self.obstaclesXYZ:
                            self.obstaclesXYZ.add((X[i], Y[i], z))
                            self.AddObstacle((X[i], Y[i], z))

        else:
            # Only update in the specified region
            xmin, ymin, zmin = updateStart
            xmax, ymax, zmax = updateBox
            xmax += xmin
            ymax += ymin
            zmax += zmin
            for z in range(zmin, zmax):
                # X: row, Y: column
                X, Y, V = sp.find(grid[z])
                for i in range(len(V)):
                    if (X[i], Y[i], z) not in self.obstaclesXYZ:
                        if xmin < X[i] < xmax and ymin < Y[i] < ymax:
                            if V[i] >= OBSTACLE_THRESHOLD:
                                # Add obstacle if not already added
                                self.obstaclesXYZ.add((X[i], Y[i], z))
                                self.AddObstacle((X[i], Y[i], z))

        # Return confirmation
        LazyTheta.times[3] += time.time_ns() - start_time
        return True

    # Create truncated boundary cuboids
    def AddObstacle(self, xyz):
        x, y, z = xyz
        self.blockedXYZ |= set([(x0, y0, z0)
                                for x0 in range(x - BOUND_X + 1, x + BOUND_X)
                                for y0 in range(y - BOUND_Y + 1, y + BOUND_Y)
                                for z0 in range(z - BOUND_Z + 1, z + BOUND_Z)])

        # # Set truncated outer box
        # set([(x0, y0, z0) for x0 in range(x - BOUND_X + 1, x + BOUND_X) for y0 in
        #      range(y - BOUND_Y + 1, y + BOUND_Y) for z0 in [z - BOUND_Z, z + BOUND_Z]]) |
        # set([(x0, y0, z0) for x0 in range(x - BOUND_X + 1, x + BOUND_X) for y0 in [y - BOUND_Y, y + BOUND_Y] for
        #      z0
        #      in range(z - BOUND_Z + 1, z + BOUND_Z)]) |
        # set([(x0, y0, z0) for x0 in [x - BOUND_X, x + BOUND_X] for y0 in range(y - BOUND_Y + 1, y + BOUND_Y) for
        #      z0
        #      in range(z - BOUND_Z + 1, z + BOUND_Z)]) |
        #
        # # Set inner edge frame (to avoid LOS errors)
        # set([(x0, y0, z0) for x0 in range(x - BOUND_X + 1, x + BOUND_X) for y0 in
        #      [y - BOUND_Y + 1, y + BOUND_Y - 1]
        #      for z0 in [z - BOUND_Z + 1, z + BOUND_Z - 1]]) |
        # set([(x0, y0, z0) for x0 in [x - BOUND_X + 1, x + BOUND_X - 1] for y0 in
        #      range(y - BOUND_Y + 2, y + BOUND_Y - 1) for z0 in [z - BOUND_Z + 1, z + BOUND_Z - 1]]) |
        # set([(x0, y0, z0) for x0 in [x - BOUND_X + 1, x + BOUND_X - 1] for y0 in
        #      [y - BOUND_Y + 1, y + BOUND_Y - 1]
        #      for z0 in range(z - BOUND_Z + 2, z + BOUND_Z - 1)]) |
        # {xyz})  # Set center too

    # ---
    # Calculate area
    # ---
    # Implementing LazyTheta*-P algorithm
    # http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf
    # Params: start and end nodes
    def ComputePath(self, xyzStart, xyzEnd):
        # Start & end nodes
        self.s_end = Node(xyzEnd)
        self.s_start = Node(xyzStart, 0, goal=self.s_end)
        self.s_start.parent = self.s_start
        # If direct line of sight between start and end: return
        if self.LineOfSight(self.s_start, self.s_end):
            self.s_end.parent = self.s_start
            return True

        # To-explore stack
        self.open = {}
        # To-explore stack as an fScore sorted list
        self.openList = SortedKeyList(key=lambda r: r.fScore)
        # To-explore xyz as unordered set (for quick neighbor lookup)
        self.openXYZ = set()
        # Explored stack
        self.closed = {}
        # Explored xyz as unordered set (for quick neighbor lookup)
        self.closedXYZ = set()

        self.open[self.s_start.xyz] = self.s_start
        self.openList.add(self.s_start)
        self.openXYZ.add(self.s_start.xyz)
        while len(self.openList) != 0:
            s = self.openList.pop(0)
            self.open.pop(s.xyz)
            self.openXYZ.remove(s.xyz)
            self.SetVertex(s)
            if s.xyz == self.s_end.xyz:
                # Path found
                self.s_end = s
                self.closed[self.s_end.xyz] = self.s_end
                self.closedXYZ.add(self.s_end.xyz)
                return True

            self.closed[s.xyz] = s
            self.closedXYZ.add(s.xyz)
            # Check 26 neighbors
            nbs = self.GetVisNeighborsXYZ(s)
            for i in range(len(nbs)):
                xyz = nbs[i][0]
                if xyz not in self.closedXYZ:
                    if xyz not in self.openXYZ:
                        # Unexplored
                        s_apos = Node(xyz, goal=self.s_end)
                    else:
                        start_time = time.time_ns()
                        # Get the node currently in list
                        s_apos = self.open[xyz]
                        LazyTheta.times[4] += time.time_ns() - start_time
                        # start_time = time.time_ns()
                        # # Get the node currently in list
                        # for index, node in enumerate(self.open):
                        #     if node.xyz == xyz:
                        #         break
                        # LazyTheta.times[4] += time.time_ns() - start_time
                        # s_apos = self.open[index]

                    self.UpdateVertex(s, s_apos)

        # Can't find path
        return False

    # Quickly check neighbors without too many lookups
    def GetVisNeighborsXYZ(self, s):
        start_time = time.time_ns()
        # Get neighboring obstacles
        obs = [((x0, y0, z0) in self.blockedXYZ)
               for x0 in [s.xyz[0] + OCC_SIZE_ZIP[0] - 1, s.xyz[0] + OCC_SIZE_ZIP[0]]
               for y0 in [s.xyz[1] + OCC_SIZE_ZIP[1] - 1, s.xyz[1] + OCC_SIZE_ZIP[1]]
               for z0 in [s.xyz[2] + OCC_SIZE_ZIP[2] - 1, s.xyz[2] + OCC_SIZE_ZIP[2]]]

        # Get neighboring nodes
        nbs = []
        # Whole plane for direct points
        i = 0
        dx = dy = dz = 0
        for n1234 in [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 4, 5), (2, 3, 6, 7), (0, 2, 4, 6), (1, 3, 5, 7)]:
            if not all([obs[j] for j in n1234]):
                if i < 2:
                    dx = 2 * i - 1
                elif i < 4:
                    dx = 0
                    dy = 2 * (i % 2) - 1
                else:
                    dy = 0
                    dz = 2 * (i % 2) - 1
                nbs.append(((s.xyz[0] + dx, s.xyz[1] + dy, s.xyz[2] + dz), NGBR_COST[0]))
            i += 1

        # Columns for diagonal points
        i = 0
        dx = dy = 0
        dz = -1
        for n12 in [(0, 2), (4, 6), (0, 4), (2, 6), (0, 1), (4, 5), (2, 3), (6, 7), (1, 3), (5, 7), (1, 5), (3, 7)]:
            if not all([obs[j] for j in n12]):
                if i < 2:
                    dx = 2 * i - 1
                elif i < 4:
                    dx = 0
                    dy = 2 * (i % 2) - 1
                elif i < 8:
                    dz = 0
                    dx = 2 * (i % 2) - 1
                    dy = 1 if i >= 6 else -1
                elif i < 10:
                    dz = 1
                    dy = 0
                    dx = 2 * (i % 2) - 1
                else:
                    dx = 0
                    dy = 2 * (i % 2) - 1
                nbs.append(((s.xyz[0] + dx, s.xyz[1] + dy, s.xyz[2] + dz), NGBR_COST[1]))
            i += 1

        # Blocks for corner points
        i = 0
        for n1 in range(8):
            if not obs[n1]:
                nbs.append(((s.xyz[0] + 2 * (i % 2) - 1,
                             s.xyz[1] + 2 * ((i // 2) % 2) - 1,
                             s.xyz[2] + 2 * ((i // 4) % 2) - 1), NGBR_COST[2]))
            i += 1

        LazyTheta.times[0] += time.time_ns() - start_time
        return nbs

    # Implementing fast ray traversal algorithm
    # http://www.cse.yorku.ca/~amana/research/grid.pdf
    def LineOfSight(self, start, end):
        # If in visible neighbors -> return True
        nbs = self.GetVisNeighborsXYZ(start)
        for nb in nbs:
            if nb[0] == end.xyz:
                return True

        # Allow LOS timing checks
        start_time = time.time_ns()

        # Ray traversing from end to start
        v = [st - en for (st, en) in zip(start.xyz, end.xyz)]
        # Check if ray lies on x/y/z plane, in which case different checks should be enabled
        aligned = [(i == 0) for i in v]
        step = np.array([round(np.sign(i)) for i in v])
        norm = np.linalg.norm(v)
        if norm < 0.001:
            LazyTheta.times[1] += time.time_ns() - start_time
            return True
        v = v / norm
        mode = 0
        if not any(aligned):
            # All is well
            # Define start & end
            startVoxel = np.array([round(end.xyz[i] + OCC_SIZE_ZIP[i] + (step[i] - 1) / 2) for i in range(3)])
            endVoxel = [round(start.xyz[i] + OCC_SIZE_ZIP[i] + (- step[i] - 1) / 2) for i in range(3)]
            endVoxelTuple = tuple(endVoxel)
            tDelta = 1 / abs(v)
            tMax = np.array(tDelta)  # starting from vertex, not crossing any boundaries
            # Check for corner moves
            doubleMove = False
            tripleMove = False
            dMAxes = []
            dMAx = 0
            if v[0] == v[1] == v[2]:
                tripleMove = True
                mode = 2
            for i in range(3):
                if startVoxel[i] == endVoxel[i]:
                    tMax[i] = math.inf
                if not tripleMove and v[i] == v[(i + 1) % 3]:
                    mode = 1
                    doubleMove = True
                    dMAxes = [i, (i + 1) % 3]
                    dMAx = (i + 2) % 3

            # Check if should prioritize double corner moves
            prioritizeDoubleMove = False
            tmp, = np.where(tDelta == tDelta.min())
            if len(tmp) == 2:
                prioritizeDoubleMove = True

            # Check for smallest tDelta
            tDeltaMin = np.argmin(tDelta)

            XYZ = startVoxel
            if tuple(XYZ) in self.blockedXYZ:
                LazyTheta.times[1] += time.time_ns() - start_time
                return False

            if tripleMove:
                while tuple(XYZ) != endVoxelTuple:
                    XYZ += step
                    if tuple(XYZ) in self.blockedXYZ:
                        LazyTheta.times[1] += time.time_ns() - start_time
                        return False

            elif doubleMove:
                while tuple(XYZ) != endVoxelTuple:
                    tmp, = np.where(tMax == tMax.min())
                    # If the two doubleMove axes are min -> prioritize
                    if len(tmp) == 2 or (len(tmp) == 3 and prioritizeDoubleMove):
                        for i in dMAxes:
                            XYZ[i] += step[i]
                            tMax[i] += tDelta[i]
                    # Else this is normal
                    else:
                        XYZ[dMAx] += step[dMAx]
                        tMax[dMAx] += tDelta[dMAx]

                    if tuple(XYZ) in self.blockedXYZ:
                        LazyTheta.times[1] += time.time_ns() - start_time
                        return False

            else:
                tMax100 = [10e6 if tMax[i] > 100000 else round(tMax[i] * 100) for i in range(3)]
                tDelta100 = [round(tDelta[i] * 100) for i in range(3)]
                tDeltaMinDual = [i if tDelta100[i] < tDelta100[(i + 1) % 3] else (i + 1) % 3 for i in range(3)]
                while tuple(XYZ) != endVoxelTuple:
                    # Need some optimization
                    # If all 3 tMax are equal -> prioritize smallest tDelta
                    if tMax100[0] == tMax100[1] == tMax100[2]:
                        XYZ[tDeltaMin] += step[tDeltaMin]
                        tMax100[tDeltaMin] += tDelta100[tDeltaMin]
                    else:
                        if tMax100[0] < tMax100[1]:
                            # Normal
                            if tMax100[0] < tMax100[2]:
                                XYZ[0] += step[0]
                                tMax100[0] += tDelta100[0]
                            elif tMax100[2] < tMax100[0]:
                                XYZ[2] += step[2]
                                tMax100[2] += tDelta100[2]
                            # If 2 are equal minimum -> prioritize smallest tDelta between the two
                            else:
                                XYZ[tDeltaMinDual[2]] += step[tDeltaMinDual[2]]
                                tMax100[tDeltaMinDual[2]] += tDelta100[tDeltaMinDual[2]]
                        elif tMax100[1] < tMax100[0]:
                            # Normal
                            if tMax100[1] < tMax100[2]:
                                XYZ[1] += step[1]
                                tMax100[1] += tDelta100[1]
                            elif tMax100[2] < tMax100[1]:
                                XYZ[2] += step[2]
                                tMax100[2] += tDelta100[2]
                            # If 2 are equal minimum -> prioritize smallest tDelta between the two
                            else:
                                XYZ[tDeltaMinDual[1]] += step[tDeltaMinDual[1]]
                                tMax100[tDeltaMinDual[1]] += tDelta100[tDeltaMinDual[1]]
                        # Normal
                        elif tMax100[2] < tMax100[0]:
                            XYZ[2] += step[2]
                            tMax100[2] += tDelta100[2]
                        # If 2 are equal minimum -> prioritize smallest tDelta between the two
                        else:
                            XYZ[tDeltaMinDual[0]] += step[tDeltaMinDual[0]]
                            tMax100[tDeltaMinDual[0]] += tDelta100[tDeltaMinDual[0]]

                    if tuple(XYZ) in self.blockedXYZ:
                        LazyTheta.times[5] += time.time_ns() - start_time
                        LazyTheta.times[1] += time.time_ns() - start_time
                        return False

            # Need some optimization
            # while tuple(XYZ) != tuple(endVoxel):
            #     if tripleMove:
            #         XYZ += step
            #     elif doubleMove:
            #         tmp, = np.where(tMax == tMax.min())
            #         # If the two doubleMove axes are min -> prioritize
            #         if len(tmp) == 2 or (len(tmp) == 3 and prioritizeDoubleMove):
            #             for i in dMAxes:
            #                 XYZ[i] += step[i]
            #                 tMax[i] += tDelta[i]
            #         # Else this is normal
            #         else:
            #             XYZ[dMAx] += step[dMAx]
            #             tMax[dMAx] += tDelta[dMAx]
            #     else:
            #         # Need some optimization
            #         tmp, = np.where(tMax == tMax.min())
            #         # If all 3 tMax are equal -> prioritize smallest tDelta
            #         if len(tmp) == 3:
            #             index = tDeltaMin
            #         # If 2 are equal minimum -> prioritize smallest tDelta between the two
            #         elif len(tmp) == 2:
            #             index = tmp[0] if tDelta[tmp[0]] < tDelta[tmp[1]] else tmp[1]
            #         # Else this is normal
            #         else:
            #             index = np.argmin(tMax)
            #         XYZ[index] += step[index]
            #         tMax[index] += tDelta[index]
            #
            #     if tuple(XYZ) in self.blockedXYZ:
            #         if not (doubleMove or tripleMove):
            #             LazyTheta.times[5] += time.time_ns() - start_time
            #         LazyTheta.times[1] += time.time_ns() - start_time
            #         return False

        elif np.count_nonzero(aligned) == 1:
            # Traverse between faces
            axes = [0, 1, 2]
            ax = axes.pop(aligned.index(True))
            # Define start & end
            # Store face of higher-valued voxel
            startFace = np.array([round(end.xyz[i] + OCC_SIZE_ZIP[i] + (step[i] - 1) / 2) if i in axes
                                  else round(end.xyz[i] + OCC_SIZE_ZIP[i]) for i in range(3)])
            endFace = [round(start.xyz[i] + OCC_SIZE_ZIP[i] + (- step[i] - 1) / 2) if i in axes
                       else round(start.xyz[i] + OCC_SIZE_ZIP[i]) for i in range(3)]
            tDelta = np.array([1.0 / abs(v[i]) if i in axes else math.inf for i in range(3)])
            tMax = np.array(tDelta)  # starting from vertex, not crossing any boundaries
            # Check for corner moves
            doubleMove = False
            if v[axes[0]] == v[axes[1]]:
                doubleMove = True
            for i in range(3):
                if startFace[i] == endFace[i]:
                    tMax[i] = math.inf

            # Check for smallest tDelta
            tDeltaMin = np.argmin(tDelta)

            extra = [0 if i != ax else 1 for i in range(3)]
            XYZ = [startFace, startFace - np.array(extra)]
            if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(2)]):
                LazyTheta.times[1] += time.time_ns() - start_time
                return False

            while tuple(XYZ[0]) != tuple(endFace):
                if doubleMove:
                    for i in range(2):
                        XYZ[i] += step
                else:
                    tmp, = np.where(tMax == tMax.min())
                    # If both tMax are equal -> prioritize smallest tDelta
                    if len(tmp) == 2:
                        index = tDeltaMin
                    # Else this is normal
                    else:
                        index = np.argmin(tMax)
                    for i in range(2):
                        XYZ[i][index] += step[index]
                    tMax[index] += tDelta[index]

                if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(2)]):
                    LazyTheta.times[1] += time.time_ns() - start_time
                    return False

        elif np.count_nonzero(aligned) == 2:
            # Traverse on axis
            axes = [0, 1, 2]
            ax = axes.pop(aligned.index(False))
            # Define start & end
            # Store edge of higher-valued voxel
            startEdge = np.array([round(end.xyz[i] + OCC_SIZE_ZIP[i] + (step[i] - 1) / 2) if i == ax
                                  else round(end.xyz[i] + OCC_SIZE_ZIP[i]) for i in range(3)])
            endEdge = [round(start.xyz[i] + OCC_SIZE_ZIP[i] + (- step[i] - 1) / 2) if i == ax
                       else round(start.xyz[i] + OCC_SIZE_ZIP[i]) for i in range(3)]
            extra = [[0, 0, 0],
                     [0 if i != axes[0] else 1 for i in range(3)],
                     [0 if i != axes[1] else 1 for i in range(3)],
                     [1 if i != ax else 0 for i in range(3)]]
            XYZ = [startEdge - np.array(extra[i]) for i in range(4)]
            if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(4)]):
                LazyTheta.times[1] += time.time_ns() - start_time
                return False

            while tuple(XYZ[0]) != tuple(endEdge):
                for i in range(4):
                    XYZ[i] += step

                if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(4)]):
                    LazyTheta.times[1] += time.time_ns() - start_time
                    return False

        if not any(aligned) and mode == 0:
            LazyTheta.times[5] += time.time_ns() - start_time
        LazyTheta.times[1] += time.time_ns() - start_time
        return True

    def SetVertex(self, s):
        if s.xyz == self.s_start.xyz:
            return True
        if not self.LineOfSight(s.parent, s):
            nbs = self.GetVisNeighborsXYZ(s)
            i = 0
            currentGCMin = math.inf
            currentXYZ = None
            for i in range(len(nbs)):
                if nbs[i][0] in self.closedXYZ:
                    start_time = time.time_ns()
                    if self.closed[nbs[i][0]].gScore + nbs[i][1] < currentGCMin:
                        currentGCMin = self.closed[nbs[i][0]].gScore + nbs[i][1]
                        currentXYZ = nbs[i][0]
                    LazyTheta.times[4] += time.time_ns() - start_time

            if currentXYZ is not None:
                s.parent = self.closed[currentXYZ]
                s.UpdateGScore(currentGCMin)
                return True

            # currentIndex = -1
            # for i in range(len(nbs)):
            #     if nbs[i][0] in self.closedXYZ:
            #         start_time = time.time_ns()
            #         for index, node in enumerate(self.closed):
            #             if node.xyz == nbs[i][0]:
            #                 break
            #         LazyTheta.times[4] += time.time_ns() - start_time
            #         if self.closed[index].gScore + nbs[i][1] < currentGCMin:
            #             currentGCMin = self.closed[index].gScore + nbs[i][1]
            #             currentIndex = index
            #
            # if currentIndex >= 0:
            #     s.parent = self.closed[index]
            #     s.UpdateGScore(currentGCMin)
            #     return True

        return True

    def ComputeCost(self, s, s_apos):
        # Path 2
        cScore = GetPTGDistance(s_apos.xyz, s.parent.xyz)
        if s.parent.gScore + cScore < s_apos.gScore:
            s_apos.parent = s.parent
            s_apos.UpdateGScore(s.parent.gScore + cScore)
            return True
        return False
        # A* & Theta* algorithm
        # # Path 2
        # if self.LineOfSight(s.parent, s_apos):
        #     cScore = math.sqrt((s_apos.xyz[0] - s.parent.xyz[0]) * (s_apos.xyz[0] - s.parent.xyz[0]) +
        #                        (s_apos.xyz[1] - s.parent.xyz[1]) * (s_apos.xyz[1] - s.parent.xyz[1]) +
        #                        (s_apos.xyz[2] - s.parent.xyz[2]) * (s_apos.xyz[2] - s.parent.xyz[2]))
        #     if s.parent.gScore + cScore < s_apos.gScore:
        #         s_apos.parent = s.parent
        #         s_apos.UpdateGScore(s.parent.gScore + cScore)
        #     return True
        # # return False
        #
        # # Path 1
        # else:
        #     cScore = math.sqrt((s_apos.xyz[0] - s.xyz[0]) * (s_apos.xyz[0] - s.xyz[0]) +
        #                        (s_apos.xyz[1] - s.xyz[1]) * (s_apos.xyz[1] - s.xyz[1]) +
        #                        (s_apos.xyz[2] - s.xyz[2]) * (s_apos.xyz[2] - s.xyz[2]))
        #     if s.gScore + cScore < s_apos.gScore:
        #         s_apos.parent = s
        #         s_apos.UpdateGScore(s.gScore + cScore)
        #     return True

    def UpdateVertex(self, s, s_apos):
        cScore = GetPTGDistance(s_apos.xyz, s.parent.xyz)
        if s.parent.gScore + cScore < s_apos.gScore:
            if s_apos.xyz in self.openXYZ:
                start_time = time.time_ns()
                s_apos2 = self.open.pop(s_apos.xyz)
                self.openXYZ.remove(s_apos.xyz)
                self.openList.remove(s_apos)
                LazyTheta.times[4] += time.time_ns() - start_time
                # Should be the same element
                if s_apos is not s_apos2:
                    print("Is not same :^ (")

            s_apos.parent = s.parent
            s_apos.UpdateGScore(s.parent.gScore + cScore)

            self.open[s_apos.xyz] = s_apos
            self.openXYZ.add(s_apos.xyz)
            self.openList.add(s_apos)
            return True
        return False

        # g_old = s_apos.gScore
        # self.ComputeCost(s, s_apos)
        # if s_apos.gScore < g_old:
        #     if s_apos.xyz in self.openXYZ:
        #         self.openXYZ.remove(s_apos.xyz)
        #         start_time = time.time_ns()
        #         for index, node in enumerate(self.openList):
        #             if node.xyz == s_apos.xyz:
        #                 break
        #         LazyTheta.times[4] += time.time_ns() - start_time
        #         s_apos.parent = self.openList.pop(index).parent
        #
        #     self.openXYZ.add(s_apos.xyz)
        #     self.openList.add(s_apos)
        #     return True
        # return False
