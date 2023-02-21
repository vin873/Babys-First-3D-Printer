import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

# subscribe each cake's position
browns = [(1125, 725), (1125, 1275), (1875, 725), (1875, 1275)]
yellows = [(775, 225), (775, 1775), (2225, 225), (2225, 1775)]
pinks = [(575, 225), (575, 1775), (2425, 225), (2425, 1775)]

tempB = [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]
tempY = [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]
tempP = [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]

# subscribe each enemy's pos
enemies = [(1125, 1775), (1875, 225)]
# subscribe our robots pos
startPos = [(1125, 225), (1125, 500)]

picked = [[(-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1)]]
used = []
got = [[0, 0, 0], [0, 0, 0]] # brown, yellow, pink
fullness = [[0, 0, 0, 0], [0, 0, 0, 0]]
currMin = [99999, 99999]
absAng = [0, 0]
minAngle = [360, 360]

def closerEnemy(target):
    global enemies
    speed = 0.9 # enemy/our
    min = 99999
    for enemy in enemies:
        if enemy != (-1, -1):
           if min > euclidean(enemy, target) / speed:
               min = euclidean(enemy, target) / speed
    return min

def whatColorGet(pos, num):
    if pos in browns:
        got[num][0] = 1
    if pos in yellows:
        got[num][1] = 1
    if pos in pinks:
        got[num][2] = 1

def closest(pos, num):
    tempMin = 99999
    tempPicked = ()
    if got[num][0] != 1:
        for i in browns:
            if tempMin > euclidean(pos, i):
                tempMin = euclidean(pos, i)
                tempPicked = i
    if got[num][1] != 1:
        for i in yellows:
            if tempMin > euclidean(pos, i):
                tempMin = euclidean(pos, i)
                tempPicked = i
    if got[num][2] != 1:
        for i in pinks:
            if tempMin > euclidean(pos, i):
                tempMin = euclidean(pos, i)
                tempPicked = i
    return tempPicked

def where2go(pos, num):
    global tempB, tempY, tempP, got, picked, enemies, currMin, fullness, absAng, minAngle, used
    tempMin = 99999
    for enemy in enemies:
        if enemy != (-1, -1):
            for target in browns:
                if euclidean(pos, target) < euclidean(enemy, target):
                    tempB[browns.index(target)] = target
            for target in yellows:
                if euclidean(pos, target) < euclidean(enemy, target):
                    tempY[yellows.index(target)] = target
            for target in pinks:
                if euclidean(pos, target) < euclidean(enemy, target):
                    tempP[pinks.index(target)] = target

    for i in used:
        if i in tempB:
            tempB[tempB.index(i)] = (-1, -1)
        elif i in tempY:
            tempY[tempY.index(i)] = (-1, -1)
        elif i in tempP:
            tempP[tempP.index(i)] = (-1, -1)

    if got[num][0] == 1:
        tempB = [-1, -1]
    if got[num][1] == 1:
        tempY = [-1, -1]
    if got[num][2] == 1:
        tempP = [-1, -1]

    print(tempB, tempY, tempP)
    orders = list(permutations([tempB, tempY, tempP]))

    for order in orders:
        for i in order[0]:
            for j in order[1]:
                for k in order[2]:
                    if order[0] == [-1, -1]:
                        i = pos
                    if order[1] == [-1, -1]:
                        j = i
                    if order[2] == [-1, -1]:
                        k = j
                    tempDis = 0
                    enemyDis = 0
                    if i != (-1, -1) and j != (-1, -1) and k != (-1, -1):
                        tempDis = euclidean(pos, i) + euclidean(i, j)
                        # print(i, j, k, tempDis)
                        enemyDis = closerEnemy(j)
                        if tempDis < enemyDis:
                            tempDis += euclidean(j, k)
                            enemyDis = closerEnemy(k)
                            if tempDis < enemyDis and tempDis < tempMin:
                                tempMin = tempDis
                                picked[num] = [i, j, k]

    if picked[num] == [(-1, -1), (-1, -1), (-1, -1)]:
        tempPos = pos
        for i in range(3):
            picked[num][i] = closest(tempPos, num)
            tempPos = picked[num][i]
            whatColorGet(picked[num][i], num)

    if picked[num]:
        tAngle = (np.rad2deg(np.arctan2(picked[num][0][1]-pos[1], picked[num][0][0]-pos[0])) - absAng[num]+360) % 360
        tAngles = []
        for i in range(4):
            if fullness[num][i] == 0:
                tAngles.append(tAngle-i*90)
        for i in tAngles:
            if abs(i) < abs(minAngle[num]):
                minAngle[num] = i
        outAngle = absAng[num] + minAngle[num]
        print(outAngle)

    print(picked[num])
    currMin[num] = tempMin

for robot in startPos:
    if robot != (-1, -1):
        where2go(robot, startPos.index(robot))

# print("picked", picked)
# print("got", got)
# print("fullness", fullness)
print("currMin", currMin)
# print("absMin", absAng)
# print("minAngle", minAngle)

if currMin[0] < currMin[1]:
    used = picked[0]
    if startPos[1] != (-1, -1):
        picked[1] = [(-1, -1), (-1, -1), (-1, -1)]
        where2go(startPos[1], 1)
else:
    used = picked[1]
    if startPos[0] != (-1, -1):
        picked[0] = [(-1, -1), (-1, -1), (-1, -1)]
        where2go(startPos[0], 0)

print("picked", picked)
