import time
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

picked = [(-1, -1), (-1, -1), (-1, -1)]
browns=[(1125,725),(1125,1275),(1875,725),(1875,1275)]
yellows=[(775,225),(775,1775),(2225,225),(2225,1775)]
pinks=[(575,225),(575,1775),(2425,225),(2425,1775)]

orders = []
bannedList= []

enemies=[(225,225),(-1,-1)]
fullness=[0,0,0,0]
startPos=(1125,225) #, (1875,1775)
currMin=99999
absAng=0
currGoal=startPos

def orderDist(currPos,order):
    global currMin
    global bannedList
    currLowestCost = 0
    tempPicked = [(-1, -1), (-1, -1), (-1, -1)]
    tempPos = currPos
    curr = []
    for color in order:
        closest_dst = 99999
        for target in color:
            curr.append(target)
            print(curr)
            if(euclidean(currPos, target))<closest_dst and curr not in bannedList:
                closest_dst=euclidean(currPos, target)
                tempPicked[order.index(color)]=target
                tempPos=target
            curr.pop()
        currPos=tempPos
        currLowestCost+=closest_dst
        curr.append(tempPicked[order.index(color)])
        if currLowestCost>currMin:
            return 99999,(-1,-1,-1)
    if currLowestCost<currMin:
        currMin=currLowestCost
    return currLowestCost,tempPicked

def where2go(currPos, tempOrders):
    global picked, currMin
    currLowestCost = 99999
    currMin=99999
    for c in tempOrders:
        tempCost,tempPicked=orderDist(currPos,c)
        if tempCost<currLowestCost:
            currLowestCost=tempCost
            picked=tempPicked
    for i in picked:
        if i == (-1,-1):
            picked.pop(picked.index(i))

def how2ban():
    global picked, startPos, enemies
    for enemy in enemies:
        tempDis=0
        currPos=startPos
        if enemy != (-1, -1):
            for orderOfColor in range(3):
                tempDis += euclidean(picked[orderOfColor], currPos)
                if euclidean(picked[orderOfColor], enemy) < tempDis:
                    tempList = []
                    for i in range(orderOfColor+1):
                        tempList.append(picked[i])
                    if tempList not in bannedList:
                        bannedList.append(tempList)
                    return 1
                currPos=picked[orderOfColor]
    return 0

candis=[]
for i in [browns, yellows, pinks]:
    if len(i)>0:
        candis.append(i)
orders=list(permutations(candis))

where2go(startPos, orders)
bannedList=[]
while how2ban():
    where2go(startPos, orders)
    how2ban()

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
