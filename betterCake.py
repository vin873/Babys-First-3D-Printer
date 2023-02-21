import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

tempB = [[(-1, -1), (-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]]
tempY = [[(-1, -1), (-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]]
tempP = [[(-1, -1), (-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]]

# subscribe each cake's position
browns = [(1125, 725), (1125, 1275), (1875, 725), (1875, 1275)]
yellows = [(775, 225), (775, 1775), (2225, 225), (2225, 1775)]
pinks = [(575, 225), (575, 1775), (2425, 225), (2425, 1775)]

# subscribe each enemy's pos
enemies = [(1875, 225), (1125, 1775)]
# subscribe our robots pos
startPos = [(1125, 225), (1125, 500)]

picked = [[(-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1)]]
used = []
got = [[0, 0, 1], [1, 1, 1]] # brown, yellow, pink
fullness = [[0, 0, 0, 0], [0, 0, 0, 0]]
currMin = [99999, 99999]
absAng = [0, 0]
minAngle = [360, 360]
outAngle = [0, 0]

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

def whatColorReverse(pos, num):
    if pos in browns:
        got[num][0] = 0
    if pos in yellows:
        got[num][1] = 0
    if pos in pinks:
        got[num][2] = 0

def safest(pos, num):
    tempColorMin = 99999
    tempColorPicked = (-1, -1)
    if got[num][0] != 1:
        for target in browns:
            if target not in used:
               if tempColorMin > euclidean(pos, target) - closerEnemy(target):
                   tempColorMin = euclidean(pos, target) - closerEnemy(target)
                   tempColorPicked = target
    if got[num][1] != 1:
        for target in yellows:
            if target not in used:
               if tempColorMin > euclidean(pos, target) - closerEnemy(target):
                   tempColorMin = euclidean(pos, target) - closerEnemy(target)
                   tempColorPicked = target
    if got[num][2] != 1:
        for target in pinks:
            if target not in used:
               if tempColorMin > euclidean(pos, target) - closerEnemy(target):
                   tempColorMin = euclidean(pos, target) - closerEnemy(target)
                   tempColorPicked = target
    print(tempColorPicked)
    return tempColorPicked

def where2go(pos, num):
    global  tempB, tempY, tempP, picked, enemies, currMin, fullness, absAng, minAngle, used, outAngle
    if got[num] == [1, 1, 1]:
        return []
    tempMin = 99999
    for enemy in enemies:
        if enemy != (-1, -1):
            if got[num][0] != 1:
                for target in browns:
                    if euclidean(pos, target) < euclidean(enemy, target):
                        tempB[num][browns.index(target)] = target
            if got[num][1] != 1:
                for target in yellows:
                    if euclidean(pos, target) < euclidean(enemy, target):
                        tempY[num][yellows.index(target)] = target
            if got[num][2] != 1:
                for target in pinks:
                    if euclidean(pos, target) < euclidean(enemy, target):
                        tempP[num][pinks.index(target)] = target

    for i in used:
        if i in tempB[num]:
            tempB[num][tempB[num].index(i)] = (-1, -1)
        if i in tempY[num]:
            tempY[num][tempY[num].index(i)] = (-1, -1)
        if i in tempP[num]:
            tempP[num][tempP[num].index(i)] = (-1, -1)

    if got[num][0] == 1:
        tempB[num] = [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]
    if got[num][1] == 1:
        tempY[num] = [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]
    if got[num][2] == 1:
        tempP[num] = [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]

    orders = list(permutations([tempB[num], tempY[num], tempP[num]]))

    print(tempB[num], tempY[num], tempP[num])

    for order in orders:
        for i in order[0]:
            for j in order[1]:
                for k in order[2]:
                    if order[0] == [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]:
                        i = pos
                    if order[1] == [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]:
                        j = i
                    if order[2] == [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]:
                        k = j
                    tempDis = 0
                    enemyDis = 0
                    if i != (-1, -1) and j != (-1, -1) and k != (-1, -1):
                        tempDis = euclidean(pos, i) + euclidean(i, j)
                        enemyDis = closerEnemy(j)
                        if tempDis < enemyDis:
                            tempDis += euclidean(j, k)
                            enemyDis = closerEnemy(k)
                            if tempDis < enemyDis and tempDis < tempMin:
                                tempMin = tempDis
                                picked[num] = [i, j, k]
                                for o in order:
                                    if o == [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]:
                                        picked[num][order.index(o)] = (-1, -1)

    if picked[num] == [(-1, -1), (-1, -1), (-1, -1)]:
        picked[num][0] = safest(pos, num)
        whatColorGet(picked[num][0], num)
        picked[num][1] = safest(pos, num)
        whatColorGet(picked[num][1], num)
        picked[num][2] = safest(pos, num)
        whatColorGet(picked[num][2], num)
        for i in range(3):
            if picked[num][i] != (-1, -1):
                whatColorReverse(picked[num][i], num)

    if picked[num]:
        tAngle = (np.rad2deg(np.arctan2(picked[num][0][1]-pos[1], picked[num][0][0]-pos[0])) - absAng[num]+360) % 360
        tAngles = []
        for i in range(4):
            if fullness[num][i] == 0:
                tAngles.append(tAngle-i*90)
        for i in tAngles:
            if abs(i) < abs(minAngle[num]):
                minAngle[num] = i
        outAngle[num] = absAng[num] + minAngle[num]

    currMin[num] = tempMin
    for i in picked[num]:
        if i == (-1, -1):
            picked[num].remove(i)


if enemies[0] == (-1, -1) and enemies[1] == (-1, -1):
    enemies[0] = (99999, 99999)

for robot in startPos:
    if robot != (-1, -1):
        where2go(robot, startPos.index(robot))
print(currMin)

if currMin[0] < currMin[1]:
    used = picked[0]
    if startPos[1] != (-1, -1):
        picked[1] = [(-1, -1), (-1, -1), (-1, -1)]
        currMin[1] = 99999
        where2go(startPos[1], 1)
else:
    used = picked[1]
    if startPos[0] != (-1, -1):
        picked[0] = [(-1, -1), (-1, -1), (-1, -1)]
        currMin[0] = 99999
        where2go(startPos[0], 0)

print("outAngle", outAngle)
print("picked", picked)
print("got", got)
# print("fullness", fullness)
# print("currMin", currMin)
# print("absMin", absAng)
# print("minAngle", minAngle)