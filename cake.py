import time
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

picked = [(-1, -1), (-1, -1), (-1, -1)]
browns=[(1125,725),(1125,1275),(1875,725),(1875,1275)]
yellows=[(775,225),(775,1775),(2225,225),(2225,1775)]
pinks=[(575,225),(575,1775),(2425,225),(2425,1775)]
enemies=[(1125,1775),(-1,-1)]
fullness=[0,0,0,0]
startPos=[(1125,225), (1875,1775)]
currMin=99999
absAng=0
currGoal=startPos

def orderDist(currPos,order):
    global currMin
    currLowestCost = 0
    tempPicked = [(-1, -1), (-1, -1), (-1, -1)]
    tempPos = currPos
    for j in order:
        closest_dst = 99999
        for c in j:
            if(euclidean(currPos, c))<closest_dst:
                closest_dst=euclidean(currPos, c)
                tempPicked[order.index(j)]=c
                tempPos=c
        currPos=tempPos
        currLowestCost+=closest_dst
        if currLowestCost>currMin:
            return 99999,(-1,-1,-1)
    print(currLowestCost)
    print(tempPicked)
    if currLowestCost<currMin:
        currMin=currLowestCost
    return currLowestCost,tempPicked

def where2go(currPos):
    global picked
    global currMin
    currLowestCost = 99999
    currMin=99999
    candis=[]
    for i in [browns, yellows, pinks]:
        if len(i)>0:
            candis.append(i)
    orders=list(permutations(candis))
    for c in orders:
        tempCost,tempPicked=orderDist(currPos,c)
        if tempCost<currLowestCost:
            currLowestCost=tempCost
            picked=tempPicked
    for i in picked:
        if i == (-1,-1):
            picked.pop(picked.index(i))

# startPos=(float(input("x: ")),float(input("y: ")))
# absAng=float(input("theta: "))
where2go(startPos)
tAngle = (np.rad2deg(np.arctan2(picked[0][1]-startPos[1],picked[0][0]-startPos[0])) - absAng+360)%360
tAngles=[]
for i in range (4):
    if fullness[i] == 0:
        tAngles.append(tAngle-i*90)
print(tAngles)
min = 360
for i in tAngles:
    if abs(i) < abs(min):
        min = i
print(min)
outAngle=absAng+min
print(outAngle)
print(picked)
