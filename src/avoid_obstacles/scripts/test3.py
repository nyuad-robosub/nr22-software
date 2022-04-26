import scipy.sparse as sp
import numpy as np
# Lengths
Xlength = 100
Ylength = 100
Zlength = 100

xlim = Xlength / 2
ylim = Ylength / 2
zlim = Zlength / 2

# Generate some obstacles
# prepare some coordinates
u, v, w = np.indices((Xlength, Ylength, Zlength), sparse=True)
# draw cuboids in the top left and bottom right corners, and a link between them
cube1 = (u > xlim - 2) & (u < xlim + 2) & (v > Ylength - 4) & (w > zlim - 2) & (w < zlim + 2)
cube2 = (u > Xlength - 4) & (v > ylim - 2) & (v < ylim + 2) & (w > Zlength - 4)
link = abs((u + 1) - (Ylength - (v - ylim + 1))) + abs((Ylength - (v - ylim + 1)) - w) + abs(w - (u + 1)) <= 2
# combine the objects into a single boolean array
voxelarray = cube1 | cube2 | link

import time
start_time = time.time_ns()
grid = []
for z in range(Zlength):
    grid.append(sp.lil_array(voxelarray[z]))
for i in range(0):
    print(i)
    # print([sp.find(grid[i]) for i in range(len(grid))])
    for z in range(Zlength):
        # print(grid[z])
        for x in range(Xlength):
            for y in range(Ylength):
                grid[z][x, y] = not grid[z][x, y]
                #print(grid[z][x, y])
print("Lil: ", (time.time_ns() - start_time) / 1000000, "ns")
start_time = time.time_ns()
grid = []
for z in range(Zlength):
    grid.append(sp.dok_array(voxelarray[z]))
for i in range(0):
    print(i)
    # print([sp.find(grid[i]) for i in range(len(grid))])
    for z in range(Zlength):
        # print(grid[z])
        for x in range(Xlength):
            for y in range(Ylength):
                grid[z][x, y] = not grid[z][x, y]
                #print(grid[z][x, y])
print("Lil: ", (time.time_ns() - start_time) / 1000000, "ns")

import random
import sys
def sizeof(obj):
    size = sys.getsizeof(obj)
    if isinstance(obj, dict): return size + sum(map(sizeof, obj.keys())) + sum(map(sizeof, obj.values()))
    if isinstance(obj, (list, tuple, set, frozenset)): return size + sum(map(sizeof, obj))
    return size
class Clown:
    def __init__(self):
        self.flag = True
start_time = time.time_ns()
XYZ_lil = []
for i in range(100):
    xyz = [random.randint(0, 999) for _ in range(3)]
    while len(XYZ_lil) < xyz[0] + 1:
        XYZ_lil.append([])
    while len(XYZ_lil[xyz[0]]) < xyz[1] + 1:
        XYZ_lil[xyz[0]].append([])
    while len(XYZ_lil[xyz[0]][xyz[1]]) < xyz[2] + 1:
        XYZ_lil[xyz[0]][xyz[1]].append(None)
    XYZ_lil[xyz[0]][xyz[1]][xyz[2]] = Clown()
print("Lolv:", (time.time_ns() - start_time) / 1000000, "ms")
print("Lolv size:", sizeof(XYZ_lil))
start_time = time.time_ns()
XYZ_dic = {}
for i in range(100):
    xyz = tuple([random.randint(0, 999) for _ in range(3)])
    XYZ_dic[xyz] = Clown()
print("Dict:", (time.time_ns() - start_time) / 1000000, "ms")
print("Dict size:", sizeof(XYZ_dic))

I, J, V = sp.find(grid[15])
print(I)
print(J)
print(V)

a = sp.coo_array([[2, 1, 3], [0, 0, 0], [7, 1, 0]], dtype=np.bool_)
print(a)