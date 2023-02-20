import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

# subscribe each cake's position
browns = [(1125, 725), (1125, 1275), (1875, 725), (1875, 1275)]
yellows = [(775, 225), (775, 1775), (2225, 225), (2225, 1775)]
pinks = [(575, 225), (575, 1775), (2425, 225), (2425, 1775)]

tempB = []
tempY = []
tempP = []

got = [0, 0, 0] # brown, yellow, pink

# subscribe each enemy's pose
enemies = (1125, 1775)
# subscribe our robots pose
startPos = (1125, 225)

picked = [(-1, -1), (-1, -1), (-1, -1)]
fullness = [0, 0, 0, 0]
currMin = 99999
absAng = 0
minAngle = 360

for target in browns:
    if euclidean(startPos, target) < euclidean(enemies, target):
        tempB.append(target)
    elif euclidean(startPos, target) > euclidean(enemies, target):
        tempB.append((-1, -1))
for target in yellows:
    if euclidean(startPos, target) < euclidean(enemies, target):
        tempY.append(target)
    elif euclidean(startPos, target) > euclidean(enemies, target):
        tempY.append((-1, -1))
for target in pinks:
    if euclidean(startPos, target) < euclidean(enemies, target):
        tempP.append(target)
    elif euclidean(startPos, target) > euclidean(enemies, target):
        tempP.append((-1, -1))

if got[0] == 1:
    tempB = [-1, -1]
if got[1] == 1:
    tempY = [-1, -1]
if got[2] == 1:
    tempP = [-1, -1]

orders=list(permutations([tempB, tempY, tempP]))

for order in orders:
    for i in order[0]:
        for j in order[1]:
            for k in order[2]:
                if order[0] == [-1, -1]:
                    i = startPos
                if order[1] == [-1, -1]:
                    j = i
                if order[2] == [-1, -1]:
                    k = j
                tempDis = 0
                enemyDis = 0
                if i != (-1, -1) and j != (-1, -1) and k != (-1, -1):
                    tempDis = euclidean(startPos, i) + euclidean(i, j)
                    enemyDis = euclidean(enemies, j)
                    if tempDis < enemyDis:
                        tempDis += euclidean(j, k)
                        enemyDis = euclidean(enemies, k)
                        if tempDis < enemyDis and tempDis < currMin:
                            currMin = tempDis
                            picked = [i, j, k]
                            for o in order:
                                if o == [-1, -1]:
                                    picked.remove(picked[order.index(o)])
if picked:
    tAngle = (np.rad2deg(np.arctan2(picked[0][1]-startPos[1], picked[0][0]-startPos[0])) - absAng+360) % 360
    tAngles = []
    for i in range(4):
        if fullness[i] == 0:
            tAngles.append(tAngle-i*90)
    for i in tAngles:
        if abs(i) < abs(minAngle):
            minAngle = i
    outAngle = absAng + minAngle
    print(outAngle)

print(picked)
