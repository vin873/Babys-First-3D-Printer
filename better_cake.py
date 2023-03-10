import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations

# subscribe each cake's position
browns = [(1125, 725), (1125, 1275), (1875, 725), (1875, 1275)]
yellows = [(775, 225), (775, 1775), (2225, 225), (2225, 1775)]
pinks = [(575, 225), (575, 1775), (2425, 225), (2425, 1775)]
allCakes = [browns, yellows, pinks]

tempAllCakes = []
for i in range(3):
    tempAllCakes.append([[(-1, -1), (-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1), (-1, -1)]])

# subscribe each enemy's pos
enemies = [(1125, 1775), (1875, 225)]
# subscribe our robots pos
startPos = [(1125, 225), (1875, 1775)]
absAng = [0, 0]

picked = [[(-1, -1), (-1, -1), (-1, -1)], [(-1, -1), (-1, -1), (-1, -1)]]
used = []
got = [[0, 0, 0], [0, 0, 0]]  # brown, yellow, pink
fullness = [[0, 0, 0, 0], [0, 0, 0, 0]]
currMin = [99999, 99999]
minAngle = 360
outAngle = [[0, 0, 0], [0, 0, 0]]

position = [-1, -1]
quaternion = Quaternion()
robotPose = PoseStamped()

pub = [rospy.Publisher('/robot1/nav_goal', PoseStamped, queue_size=100), rospy.Publisher('/robot2/nav_goal', PoseStamped, queue_size=100)]
stop = False

def finishcallback(msg):
    global stop
    stop = msg.data
    print(stop)

def robotPublish(num):
    global robotPose, pub
    robotPose.header.frame_id = "/robot" + str(num+1) + "/map"
    robotPose.header.stamp = rospy.Time.now()
    robotPose.pose.position.x = position[0]
    robotPose.pose.position.y = position[1]
    robotPose.pose.orientation.x = quaternion.x
    robotPose.pose.orientation.y = quaternion.y
    robotPose.pose.orientation.z = quaternion.z
    robotPose.pose.orientation.w = quaternion.w
    rospy.sleep(0.3)
    pub[num].publish(robotPose)

def euler2quaternion(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(qx, qy, qz, qw)

def howLong(pos):
    dis = 0
    for i in range(len(pos) - 1):
        dis += euclidean(pos[i], pos[i + 1])
    return dis

def closerEnemy(target):
    global enemies
    speed = 0.9  # enemy/our
    min = 99999
    for enemy in enemies:
        if enemy != (-1, -1):
            if min > euclidean(enemy, target) / speed:
                min = euclidean(enemy, target) / speed
    return min

def whatColorGet(pos, num):
    for i in range(3):
        if pos in allCakes[i]:
            got[num][i] = 1

def whatColorReverse(pos, num):
    for i in range(3):
        if pos in allCakes[i]:
            got[num][i] = 0

def safest(pos, num):
    tempColorMin = 99999
    tempColorPicked = (-1, -1)
    for i in range(3):
        if got[num][i] != 1:
            for target in allCakes[i]:
                if target not in used:
                    if tempColorMin > euclidean(pos, target) - closerEnemy(target):
                        tempColorMin = euclidean(pos, target) - closerEnemy(target)
                        tempColorPicked = target
    # print(tempColorPicked)
    return tempColorPicked

def where2go(pos, num):
    global picked, enemies, currMin, fullness, absAng, minAngle, used, outAngle
    if got[num] == [1, 1, 1]:
        return []
    tempMin = 99999
    for enemy in enemies:
        if enemy != (-1, -1):
            for i in range(3):
                if got[num][i] != 1:
                    for target in allCakes[i]:
                        if euclidean(pos, target) < euclidean(enemy, target):
                            tempAllCakes[i][num][allCakes[i].index(target)] = target

    for i in used:
        for j in range(3):
            if i in tempAllCakes[j][num]:
                tempAllCakes[j][num][tempAllCakes[j][num].index(i)] = (-1, -1)

    for i in range(3):
        if got[num][i] == 1:
            tempAllCakes[i][num] = [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]

    orders = list(permutations([tempAllCakes[0][num], tempAllCakes[1][num], tempAllCakes[2][num]]))
    # print(tempB[num], tempY[num], tempP[num])

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
                        if tempDis < enemyDis and tempDis < tempMin:
                            tempDis += euclidean(j, k)
                            enemyDis = closerEnemy(k)
                            if tempDis < enemyDis and tempDis < tempMin:
                                tempMin = tempDis
                                picked[num] = [i, j, k]
                                # print(num, picked[num])
                                for o in order:
                                    if o == [(99999, 99999), (99999, 99999), (99999, 99999), (99999, 99999)]:
                                        picked[num][order.index(o)] = (-1, -1)

    if picked[num] == [(-1, -1), (-1, -1), (-1, -1)]:
        # print("so safe", num)
        picked[num][0] = safest(pos, num)
        whatColorGet(picked[num][0], num)
        picked[num][1] = safest(picked[num][0], num)
        whatColorGet(picked[num][1], num)
        picked[num][2] = safest(picked[num][1], num)
        whatColorGet(picked[num][2], num)
        minDis = 99999
        safeOrders = list(permutations(picked[num]))
        for order in safeOrders:
            order = list(order)
            order.insert(0, pos)
            if minDis > howLong(order):
                minDis = howLong(order)
                order.remove(pos)
                picked[num] = order
        for i in range(3):
            if picked[num][i] != (-1, -1):
                whatColorReverse(picked[num][i], num)

    for i in range(3):
        if picked[num][i] == (-1, -1):
            picked[num].remove(picked[num][i])
    if picked[num]:
        anglePos = pos
        tempAng = list(absAng)
        tempFull = list(fullness[num])
        for j in range(3):
            minAngle = 360
            minAngleNum = -1
            tAngle = (np.rad2deg(np.arctan2(picked[num][j][1] - anglePos[1], picked[num][j][0] - anglePos[0])) - absAng[num] + 360) % 360
            tAngles = [360, 360, 360, 360]
            for i in range(4):
                if tempFull[i] == 0:
                    tAngles[i] = (tAngle - i * 90 - tempAng[num] + 360) % 360
                    if tAngles[i] > 180:
                        tAngles[i] -= 360
            for i in tAngles:
                if abs(i) < abs(minAngle):
                    minAngle = i
                    minAngleNum = tAngles.index(i)
            outAngle[num][j] = minAngle + tempAng[num]
            # print(outAngle[num], tempAng[num], tAngles, minAngle, tAngle)
            tempAng[num] = outAngle[num][j]
            tempFull[minAngleNum] = 1
            anglePos = picked[num][j]

    currMin[num] = tempMin

def publisher():
    global quaternion, robotPose
    rospy.init_node("better_cake")
    rospy.Subscriber("/finishornot", Bool, finishcallback)
    pub = rospy.Publisher('where2go', Float64, queue_size = 100)
    
    if enemies[0] == (-1, -1) and enemies[1] == (-1, -1):
        enemies[0] = (99999, 99999)

    for robot in startPos:
        if robot != (-1, -1):
            where2go(robot, startPos.index(robot))

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

    for robot in range(2):
        if startPos[robot] != (-1, -1):
            print("robot", robot)
            for pos in range(3):
                for axis in range(2):
                    position[axis] = picked[robot][pos][axis] * 0.001
                print(outAngle[robot][pos])
                quaternion = euler2quaternion(0, 0, outAngle[robot][pos] * math.pi / 180)
                robotPublish(robot)
                print(position)
                print(quaternion)
                rospy.sleep(5)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass


# print("outAngle", outAngle)
# print("picked", picked)

# for robot in picked:
#     print("robot" + str(picked.index(robot)) + " : ")
#     for target in robot:
#         for i in range(3):
#             if target in allCakes[i]:
#                 print('[' + str(i) + ']' + '[' + str(allCakes[i].index(target)) + ']')

# print("got", got)
# print("fullness", fullness)
# print("currMin", currMin)
# print("absMin", absAng)
# print("minAngle", minAngle)

# Test1 = [startPos[0], pinks[0], yellows[0], browns[1]]
# Test2 = [startPos[0], yellows[0], pinks[0], browns[1]]
# print(howLong(Test1))
# print(howLong(Test2))
