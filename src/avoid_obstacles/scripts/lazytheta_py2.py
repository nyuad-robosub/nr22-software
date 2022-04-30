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
OCC_SIZE_X = 150
OCC_SIZE_Y = 40
OCC_SIZE_Z = 30
OCC_SIZE_ZIP = [int(round(OCC_SIZE_X / 2)), int(round(OCC_SIZE_Y / 2)), int(round(OCC_SIZE_Z / 2))]
# Unit dimensions of occupancy grid (meters)
OCC_UNIT_X = OCC_UNIT_Y = OCC_UNIT_Z = OCC_UNIT = 0.1
# Determining characteristics of obstacle definition
OBSTACLE_THRESHOLD = 255
# Define boundaries for pool surfaces
MAX_HEIGHT = OCC_SIZE_Z / 2
MAX_DEPTH = - OCC_SIZE_Z / 2
# Define boundaries for obstacles (truncated box)
AVOIDANCE_RADIUS = 0.5 # meters
BOUND_X = BOUND_Y = BOUND_Z = BOUND = int(round(AVOIDANCE_RADIUS / OCC_UNIT))
# Get costs of neighbors
NGBR_COST = (1, 1.41421, 1.73205)

# For analytics
USE_PTG_TYPE = 0

# Different schemes to find Pythagoras distance
# Integer parameters
def GetPTGDistance(xyz_start, xyz_end):
    if USE_PTG_TYPE == 0:
        start_time = time.time()
        val = math.sqrt((xyz_start[0] - xyz_end[0]) ** 2 +
                         (xyz_start[1] - xyz_end[1]) ** 2 +
                         (xyz_start[2] - xyz_end[2]) ** 2)
        LazyTheta.times[2] += time.time() - start_time
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
    def __init__(self, xyz=(0, 0, 0), gScore=1e9, goal=None, parent=None):
        self.xyz = xyz
        self.gScore = gScore
        self.hScore = 0
        self.fScore = gScore
        self.parent = parent

        if goal is not None:
            self.UpdateHScore(goal)

    def UpdateGScore(self, gScore=1e9):
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
        # self.grid = []
        # for z in range(OCC_SIZE_Z):
        #     # X: row, Y: column
        #     self.grid.append(sp.lil_matrix((OCC_SIZE_X, OCC_SIZE_Y), dtype=np.uint8))

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

    # Update occupancy set, with updateStart and updateBox tuples of 3D ints
    def UpdateOccupancySet(self, occSet=None, updateStart=None, updateBox=None):
        start_time = time.time()
        # Update occupancy grid, with updateStart and updateBox tuples of 3D ints
        if occSet is None:
            return False

        # If updateStart is None: function called to initialize obstacles
        if updateStart is None:
            self.obstaclesXYZ = set()
            self.blockedXYZ = set()
            for x in range(OCC_SIZE_X):
                for y in range(OCC_SIZE_Y):
                    for z in range(self.zMin, self.zMax):
                        # Add obstacle if not already added
                        if (x, y, z) in occSet:
                            xyz_ = tuple(np.array([x, y, z]) - np.array(OCC_SIZE_ZIP))
                            if xyz_ not in self.obstaclesXYZ:
                                self.obstaclesXYZ.add(xyz_)
                                self.AddObstacle(xyz_)

        else:
            # Only update in the specified region
            xmin, ymin, zmin = updateStart
            xmax, ymax, zmax = updateBox
            xmax += xmin
            ymax += ymin
            zmax += zmin
            for x in range(int(round(xmin + 0.5)), int(round(xmax - 0.5))):
                for y in range(int(round(ymin + 0.5)), int(round(ymax - 0.5))):
                    for z in range(int(round(zmin + 0.5)), int(round(zmax - 0.5))):
                        # Add obstacle if not already added
                        if (x, y, z) in occSet:
                            xyz_ = tuple(np.array([x, y, z]) - np.array(OCC_SIZE_ZIP))
                            if xyz_ not in self.obstaclesXYZ:
                                self.obstaclesXYZ.add(xyz_)
                                self.AddObstacle(xyz_)

        # Return confirmation
        LazyTheta.times[3] += time.time() - start_time
        return True

    # Create truncated boundary cuboids
    def AddObstacle(self, xyz):
        x, y, z = xyz
        self.blockedXYZ |= set([(x0, y0, z0)
                                for x0 in range(x - BOUND_X + 1, x + BOUND_X)
                                for y0 in range(y - BOUND_Y + 1, y + BOUND_Y)
                                for z0 in range(z - BOUND_Z + 1, z + BOUND_Z)])

    # ---
    # Calculate area
    # ---
    # Implementing LazyTheta*-P algorithm
    # http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf
    # Params: start and end nodes
    def ComputePath(self, xyzStart, xyzEnd):
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

        # Start & end nodes
        self.s_end = Node(xyzEnd)
        self.s_start = Node(xyzStart, 0, goal=self.s_end)
        self.s_start.parent = self.s_start
        # If direct line of sight between start and end: return
        if self.LineOfSight(self.s_start, self.s_end):
            self.s_end.parent = self.s_start
            return True

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
                        start_time = time.time()
                        # Get the node currently in list
                        s_apos = self.open[xyz]
                        LazyTheta.times[4] += time.time() - start_time
                        # start_time = time.time()
                        # # Get the node currently in list
                        # for index, node in enumerate(self.open):
                        #     if node.xyz == xyz:
                        #         break
                        # LazyTheta.times[4] += time.time() - start_time
                        # s_apos = self.open[index]

                    self.UpdateVertex(s, s_apos)

        # Can't find path
        return False

    # Quickly check neighbors without too many lookups
    def GetVisNeighborsXYZ(self, s):
        start_time = time.time()
        # Get neighboring obstacles
        obs = [((x0, y0, z0) in self.blockedXYZ)
               for x0 in [s.xyz[0] - 1, s.xyz[0]]
               for y0 in [s.xyz[1] - 1, s.xyz[1]]
               for z0 in [s.xyz[2] - 1, s.xyz[2]]]

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
                nbs.append(((s.xyz[0] + 2 * ((i // 4) % 2) - 1,
                             s.xyz[1] + 2 * ((i // 2) % 2) - 1,
                             s.xyz[2] + 2 * (i % 2) - 1), NGBR_COST[2]))
            i += 1

        LazyTheta.times[0] += time.time() - start_time
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
        start_time = time.time()

        # Ray traversing from end to start
        v = [float(st - en) for (st, en) in zip(start.xyz, end.xyz)]
        # Check if ray lies on x/y/z plane, in which case different checks should be enabled
        aligned = [(i == 0) for i in v]
        step = np.array([int(round(np.sign(i))) for i in v])
        norm = np.linalg.norm(v)
        if norm < 0.001:
            LazyTheta.times[1] += time.time() - start_time
            return True
        v = v / norm
        mode = 0
        if not any(aligned):
            # All is well
            # Define start & end
            startVoxel = np.array([int(round(end.xyz[i] + (step[i] - 1) * 1.0 / 2)) for i in range(3)])
            endVoxel = [int(round(start.xyz[i] + (- step[i] - 1) * 1.0 / 2)) for i in range(3)]
            endVoxelTuple = tuple(endVoxel)
            tDeltaInt = [int(round(150 / abs(v[i]))) for i in range(3)]
            tMaxInt = [tDeltaInt[i] for i in range(3)] # starting from vertex, not crossing any boundaries
            # Check for corner moves
            doubleMove = False
            tripleMove = False
            # Check if should prioritize double corner moves
            prioritizeDoubleMove = False
            dMAxes = []
            dMAx = 0
            if v[0] == v[1] == v[2]:
                tripleMove = True
                mode = 2
            for i in range(3):
                if startVoxel[i] == endVoxel[i]:
                    tMaxInt[i] = 1e6
                if not tripleMove and v[i] == v[(i + 1) % 3]:
                    mode = 1
                    doubleMove = True
                    dMAxes = [i, (i + 1) % 3]
                    dMAx = (i + 2) % 3

                    prioritizeDoubleMove = tDeltaInt[dMAxes[0]] < tDeltaInt[dMAx]

            # Check for smallest tDelta
            tDeltaMin = np.argmin(tDeltaInt)

            XYZ = startVoxel
            if tuple(XYZ) in self.blockedXYZ:
                LazyTheta.times[1] += time.time() - start_time
                return False

            if tripleMove:
                while tuple(XYZ) != endVoxelTuple:
                    XYZ += step
                if tuple(XYZ) in self.blockedXYZ:
                    LazyTheta.times[1] += time.time() - start_time
                    return False

            elif doubleMove:
                while tuple(XYZ) != endVoxelTuple:
                    # If the two doubleMove axes are min -> prioritize
                    if tMaxInt[dMAxes[0]] < tMaxInt[dMAx] or \
                            (tMaxInt[dMAxes[0]] == tMaxInt[dMAx] and prioritizeDoubleMove):
                        for i in dMAxes:
                            XYZ[i] += step[i]
                            tMaxInt[i] += tDeltaInt[i]
                    # Else this is normal
                    else:
                        XYZ[dMAx] += step[dMAx]
                        tMaxInt[dMAx] += tDeltaInt[dMAx]

                    if tuple(XYZ) in self.blockedXYZ:
                        LazyTheta.times[1] += time.time() - start_time
                        return False

            else:
                tDeltaMinDual = [i if tDeltaInt[i] < tDeltaInt[i - 1] else (i + 2) % 3 for i in range(3)]
                while tuple(XYZ) != endVoxelTuple:
                    # Need some optimization
                    # If all 3 tMax are equal -> prioritize smallest tDelta
                    if tMaxInt[0] == tMaxInt[1] == tMaxInt[2]:
                        XYZ[tDeltaMin] += step[tDeltaMin]
                        tMaxInt[tDeltaMin] += tDeltaInt[tDeltaMin]
                    else:
                        if tMaxInt[0] < tMaxInt[1]:
                            # Normal
                            if tMaxInt[0] < tMaxInt[2]:
                                XYZ[0] += step[0]
                                tMaxInt[0] += tDeltaInt[0]
                            elif tMaxInt[2] < tMaxInt[0]:
                                XYZ[2] += step[2]
                                tMaxInt[2] += tDeltaInt[2]
                            # If 2 are equal minimum -> prioritize smallest tDelta between the two
                            else:
                                XYZ[tDeltaMinDual[0]] += step[tDeltaMinDual[0]]
                                tMaxInt[tDeltaMinDual[0]] += tDeltaInt[tDeltaMinDual[0]]
                        elif tMaxInt[1] < tMaxInt[0]:
                            # Normal
                            if tMaxInt[1] < tMaxInt[2]:
                                XYZ[1] += step[1]
                                tMaxInt[1] += tDeltaInt[1]
                            elif tMaxInt[2] < tMaxInt[1]:
                                XYZ[2] += step[2]
                                tMaxInt[2] += tDeltaInt[2]
                            # If 2 are equal minimum -> prioritize smallest tDelta between the two
                            else:
                                XYZ[tDeltaMinDual[2]] += step[tDeltaMinDual[2]]
                                tMaxInt[tDeltaMinDual[2]] += tDeltaInt[tDeltaMinDual[2]]
                        # Normal
                        elif tMaxInt[2] < tMaxInt[0]:
                            XYZ[2] += step[2]
                            tMaxInt[2] += tDeltaInt[2]
                        # If 2 are equal minimum -> prioritize smallest tDelta between the two
                        else:
                            XYZ[tDeltaMinDual[1]] += step[tDeltaMinDual[1]]
                            tMaxInt[tDeltaMinDual[1]] += tDeltaInt[tDeltaMinDual[1]]

                    if tuple(XYZ) in self.blockedXYZ:
                        LazyTheta.times[5] += time.time() - start_time
                        LazyTheta.times[1] += time.time() - start_time
                        return False

        elif np.count_nonzero(aligned) == 1:
            # Traverse between faces
            axes = [0, 1, 2]
            ax = axes.pop(aligned.index(True))
            # Define start & end
            # Store face of higher-valued voxel
            startFace = np.array([int(round(end.xyz[i] + (step[i] - 1) * 1.0 / 2)) if i in axes
                                  else int(round(end.xyz[i])) for i in range(3)])
            endFace = [int(round(start.xyz[i] + (- step[i] - 1) * 1.0 / 2)) if i in axes
                       else int(round(start.xyz[i])) for i in range(3)]
            endFaceTuple = tuple(endFace)
            tDeltaInt = [int(round(150 / abs(v[i]))) if i in axes else 1e6 for i in range(3)]
            tMaxInt = [tDeltaInt[i] for i in range(3)] # starting from vertex, not crossing any boundaries
            # Check for corner moves
            doubleMove = False
            if v[axes[0]] == v[axes[1]]:
                doubleMove = True
            for i in range(3):
                if startFace[i] == endFace[i]:
                    tMaxInt[i] = 1e6

            # Check for smallest tDelta
            tDeltaMin = np.argmin(tDeltaInt)

            extra = [0 if i != ax else 1 for i in range(3)]
            XYZ = [startFace, startFace - np.array(extra)]
            if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(2)]):
                LazyTheta.times[1] += time.time() - start_time
                return False

            if doubleMove:
                while tuple(XYZ[0]) != endFaceTuple:
                    for i in range(2):
                        XYZ[i] += step

                    if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(2)]):
                        LazyTheta.times[1] += time.time() - start_time
                        return False

            else:
                # If both tMax are equal -> prioritize smallest tDelta -> assign to axes[0]
                if axes[0] != tDeltaMin:
                    axes[1] = axes[0]
                    axes[0] = tDeltaMin
                while tuple(XYZ[0]) != endFaceTuple:
                    if tMaxInt[axes[1]] < tMaxInt[axes[0]]:
                        for i in range(2):
                            XYZ[i][axes[1]] += step[axes[1]]
                        tMaxInt[axes[1]] += tDeltaInt[axes[1]]
                    else:
                        for i in range(2):
                            XYZ[i][axes[0]] += step[axes[0]]
                        tMaxInt[axes[0]] += tDeltaInt[axes[0]]

                    if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(2)]):
                        LazyTheta.times[1] += time.time() - start_time
                        return False

        elif np.count_nonzero(aligned) == 2:
            # Traverse on axis
            axes = [0, 1, 2]
            ax = axes.pop(aligned.index(False))
            # Define start & end
            # Store edge of higher-valued voxel
            startEdge = np.array([int(round(end.xyz[i] + (step[i] - 1) * 1.0 / 2)) if i == ax
                                  else int(round(end.xyz[i])) for i in range(3)])
            endEdge = [int(round(start.xyz[i] + (- step[i] - 1) * 1.0 / 2)) if i == ax
                       else int(round(start.xyz[i])) for i in range(3)]
            endEdgeTuple = tuple(endEdge)
            extra = [[0, 0, 0],
                     [0 if i != axes[0] else 1 for i in range(3)],
                     [0 if i != axes[1] else 1 for i in range(3)],
                     [1 if i != ax else 0 for i in range(3)]]
            XYZ = [startEdge - np.array(extra[i]) for i in range(4)]
            if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(4)]):
                LazyTheta.times[1] += time.time() - start_time
                return False

            while tuple(XYZ[0]) != endEdgeTuple:
                for i in range(4):
                    XYZ[i] += step

                if all([(tuple(XYZ[i]) in self.blockedXYZ) for i in range(4)]):
                    LazyTheta.times[1] += time.time() - start_time
                    return False

        if not any(aligned) and mode == 0:
            LazyTheta.times[5] += time.time() - start_time
        LazyTheta.times[1] += time.time() - start_time
        return True

    def SetVertex(self, s):
        if s.xyz == self.s_start.xyz:
            return True
        if not self.LineOfSight(s.parent, s):
            nbs = self.GetVisNeighborsXYZ(s)
            i = 0
            currentGCMin = 1e9
            currentXYZ = None
            for i in range(len(nbs)):
                if nbs[i][0] in self.closedXYZ:
                    start_time = time.time()
                    if self.closed[nbs[i][0]].gScore + nbs[i][1] < currentGCMin:
                        currentGCMin = self.closed[nbs[i][0]].gScore + nbs[i][1]
                        currentXYZ = nbs[i][0]
                    LazyTheta.times[4] += time.time() - start_time

            if currentXYZ is not None:
                s.parent = self.closed[currentXYZ]
                s.UpdateGScore(currentGCMin)
                return True
        return True

    def ComputeCost(self, s, s_apos):
        # Path 2
        cScore = GetPTGDistance(s_apos.xyz, s.parent.xyz)
        if s.parent.gScore + cScore < s_apos.gScore:
            s_apos.parent = s.parent
            s_apos.UpdateGScore(s.parent.gScore + cScore)
            return True
        return False

    def UpdateVertex(self, s, s_apos):
        cScore = GetPTGDistance(s_apos.xyz, s.parent.xyz)
        if s.parent.gScore + cScore < s_apos.gScore:
            if s_apos.xyz in self.openXYZ:
                start_time = time.time()
                s_apos2 = self.open.pop(s_apos.xyz)
                self.openXYZ.remove(s_apos.xyz)
                self.openList.remove(s_apos)
                LazyTheta.times[4] += time.time() - start_time
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
