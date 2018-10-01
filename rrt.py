import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import collections  as matColl
import math
import random
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''
def normalize(v):
    norm = np.linalg.norm(v, ord=1)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

def makeSegemnt(p1, p2, line):
    seg = dict()
    seg['p1'] = p1
    seg['p2'] = p2
    seg['l'] = line
    return seg

def pointDistance(a, b):
    return np.linalg.norm(a-b)

def dotProduct(v1, v2):
    return sum(x*y for x,y in zip(v1,v2))

def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single polygon
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

'''
Render the problem
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()

'''
Helper Methods
'''
def crossMultiply(a, b):
    c = [a[0]*b[1] - a[1]*b[0],
         a[1]*b[0] - a[0]*b[1]]
    return c

def isPointColliding(polygons, point): #checks if point is in an obstacle
    point1 = point
    point2 = [10,point[1]]
    ray = [[point1[0],point1[1]],[point2[0],point2[1]]]
    c = 0
    d = 0
    while d < len(polygons):
        obs = polygons[d]
        d+=1

        i = 0
        while i < len(obs):
            pt1 = obs[i]

            nextindex = (i+1) % len(obs)
            segment = [[pt1[0],pt1[1]],[obs[nextindex][0],obs[nextindex][1]]]
            i+=1
            if isEdgeIntersecting(segment, ray):
                c = c + 1
    if (c % 2) == 0:
        return False
    else:
        return True

def isEdgeIntersecting(l1, l2):
    p1 = [l1[0][0],l1[0][1]]
    q1 = [l1[1][0],l1[1][1]]
    p2 = [l2[0][0],l2[0][1]]
    q2 = [l2[1][0],l2[1][1]]

    o1 = direction(p1, q1, p2);
    o2 = direction(p1, q1, q2);
    o3 = direction(p2, q2, p1);
    o4 = direction(p2, q2, q1);
 
    # General case
    if (o1 != o2 and o3 != o4):
        return True;

    if (o1 == 0 and onLine(p1, p2, q1) or o2 == 0 and onLine(p1, q2, q1) or o3 == 0 and onLine(p2, p1, q2) or o4 == 0 and onLine(p2, q1, q2)):
        return True;
 
    return False; 

def direction( p, q, r):

    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
 
    if (val == 0):
        return 0;  # colinear
 
    if val > 0 :
        return 1
    else:
        return 2

def onLine( p,  q,  r):

    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
       return True;
 
    return False;

def RobotCollision(segment, robot, obstacles):
    checkList = []
    obstacleList = [[[10,10],[10,0]], [[0,0],[10,0]], [[0,10],[10,10]], [[0,0],[0,10]]]
    p1,p2 = segment['l']

    if isCollisionFree(robot, p1, obstacles) and isCollisionFree(robot, p2, obstacles):
        pass;
    else:
        return True;

    counter = 0
    while counter < len(robot):
        rt = robot[counter]
        counter+=1
        robot_offset = [(rt[0] - robot[0][0]),(rt[1] - robot[0][1])]
        checkList.append([[p1[0] + robot_offset[0], p1[1] + robot_offset[1]],[p2[0] + robot_offset[0], p2[1] + robot_offset[1]]])

    lep = 0    
    while lep < len(obstacles):
        obstacle = obstacles[lep]
        lep+=1

        i = 0
        while i < len(obstacle):
            pt1 = obstacle[i]
            i+=1
        
            nextindex = (i+1)%len(obstacle)
            z = [pt1[0],pt1[1]]
            b = [obstacle[nextindex][0],obstacle[nextindex][1]]
            obstacleList.append([z,b])

    lep = 0
    while lep < len(checkList):
        rl = checkList[lep]
        lep += 1

        etc = 0 
        while etc < len(obstacleList):
            e = obstacleList[etc]
            etc += 1
            if not isEdgeIntersecting(rl,e):
               pass
            else:
                return True
    return False

def ShortestDistToPoint(point1, point2, point3):

    point1 = np.array(point1)
    point2 = np.array(point2)
    point3 = np.array(point3)

    if(dotProduct(point3 - point1,point2 - point1) > 0 and dotProduct(point1 - point2,point3 - point2) > 0): 

        pip = dotProduct(point3 - point1,point2 - point1) * (1/pointDistance(point2, point1))
        angle = np.degrees(math.acos(dotProduct(point2 - point1, np.array([1,0]) - np.array([0,0])) * 1/(pointDistance(point2, point1) * pointDistance(np.array([1,0]), np.array([0,0])))))
        distance = math.sqrt(math.pow(pointDistance(point3, point1), 2) + (-1 * math.pow(pip,2)))

        if ((point2[1] - point1[1]) * 1) >= 0:
            nP = [pip * math.cos(np.deg2rad(angle)), pip * math.sin(np.deg2rad(angle))] - (-1*point1)
        else:
            nP = point1 - [pip * math.cos(np.deg2rad(180 - angle)), pip * math.sin(np.deg2rad(180 - angle))]

        return nP, True, distance

    else: 
        distPoint1 = pointDistance(point3, point1)
        distPoint2 = pointDistance(point3, point2)

        if distPoint1 > distPoint2:
            return point2, False, distPoint2
        else:
            return point1, False, distPoint1

'''
Grow a simple RRT
'''
def growSimpleRRT(points):

    newPoints = dict()
    adjListMap = dict()

    segments = []
    for pi in range(0,len(points) + 1):

        if pi == 0 :
            newPoints[1] = points[1]
            newPoints[2] = points[2]
            adjListMap[1] = [2]
            adjListMap[2] = []

            segments.append(makeSegemnt(1,2,[points[1],points[2]]))

        if pi < 3 :
            continue

        index = len(newPoints)+1
        newPoints[index] = points[pi]
        adjListMap[index] = []

        minDist = float("inf")
        minIndex = -1
        minPoint = [-1,-1]
        isPointGenerated = False
        minp1 = []
        minp2 = []
        count = 0

        while(count < len(segments)):
            s = segments[count]
            count+=1

            cp, nPoint, distance = ShortestDistToPoint(s['l'][0], s['l'][1], points[pi])

            if np.array_equal(cp,s['l'][0]):
                ci = s['p1']
            elif np.array_equal(cp,s['l'][1]):
                ci = s['p2']
            else:
                ci = [-1,-1]

            if distance < minDist:
                minDist = distance
                minIndex = ci
                minPoint = cp
                isPointGenerated = nPoint
                minp1 = s['p1']
                minp2 = s['p2']

        if isPointGenerated:
            newIndex = len(newPoints)+1
            newPoints[newIndex] = minPoint

            adjListMap[newIndex] = [index]

            segments.append(makeSegemnt(newIndex, index, [newPoints[newIndex], newPoints[index]]))

            if not minp2 in adjListMap[minp1]:

                if minp1 in adjListMap[minp2]:
                    adjListMap[minp2].remove(minp1)
                    adjListMap[minp2].append(newIndex)
                    adjListMap[newIndex].append(minp1)

                    for index, segment in enumerate(segments):
                        if segment['p1'] == minp2 and segment['p2'] == minp1:
                            del(segments[index])

                    
                    segments.append(makeSegemnt(minp2, newIndex, newPoints[newIndex]))

                    segments.append(makeSegemnt(newIndex, minp1, [newPoints[newIndex],newPoints[minp1]]))

            else:
                adjListMap[minp1].remove(minp2)

                for index, segment in enumerate(segments):
                    if segment['p1'] == minp1 and segment['p2'] == minp2:
                        del(segments[index])

                adjListMap[minp1].append(newIndex)
                adjListMap[newIndex].append(minp2)

                segments.append(makeSegemnt(minp1, newIndex, [newPoints[minp1],newPoints[newIndex]]))

                segments.append(makeSegemnt(newIndex, minp2, [newPoints[newIndex],newPoints[minp2]]))

        else: 
            adjListMap[minIndex].append(index)
            segments.append(makeSegemnt(minIndex, index, [newPoints[minIndex],newPoints[index]]))

    return newPoints, adjListMap

'''
Perform basic search
'''
def basicSearch(tree, start, goal):
    path = []
    stack = []
    cl = dict()

    stack.append((start,None))
    while stack:
        curr, parent_node = stack.pop()
        if(curr not in cl):
            cl[curr] = parent_node
            if curr == goal:
              break;
            
            for neighbor in tree[curr]:
                if not neighbor in cl:
                    stack.append((neighbor,curr))
    index = goal

    while True:
        path.insert(0, index)
        index = cl.get(index, None)
        if index == None:
            break

    
    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):

    polys = False

    if polygons != None:
        polys = True

    fig, ax = setupPlot()

    doot = len(path) - 1

    lines = []
    plines = []

    if polys:
        patch = createPolygonPatch(robotStart, '#66BB6A')
        ax.add_patch(patch)
        patch = createPolygonPatch(robotGoal, '#c62828')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'grey')
            ax.add_patch(patch)


    for x in tree:
        for y in tree[x]:
            p1 = points[x]
            p2 = points[y]
            lines.append([[p1[0]/10.00, p1[1]/10.00], [p2[0]/10.00, p2[1]/10.00]])

    for i in range(0,doot):
            p1 = points[path[i]]
            p2 = points[path[i+1]]
            plines.append([[points[path[i]][0]/10.00, points[path[i]][1]/10.00], [p2[0]/10.00, p2[1]/10.00]])

    ax.add_collection(matColl.LineCollection(lines, colors='#000000', linewidths=1))
    ax.add_collection(matColl.LineCollection(plines, colors='#FF4500', linewidths=1))

    plt.show()
    return

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):
    obstacleList = [[[0,0],[0,10]], [[0,10],[10,10]], [[10,10],[10,0]], [[0,0],[10,0]]]

    for rpoint in robot:
        if isPointColliding(obstacles, (point[0] + rpoint[0], point[1] + rpoint[1])):
            return False

    for obstacle in obstacles:
        for index,pt1 in enumerate(obstacle):
            nextindex = (index+1)%len(obstacle)
            obstacleList.append([[pt1[0],pt1[1]],[obstacle[nextindex][0],obstacle[nextindex][1]]])

    for i,p in enumerate(robot):
        nexti = (i+1) % len(robot)
        roboedge = [[point[0] + p[0],point[1] + p[1]],[point[0] +robot[nexti][0],point[1] + robot[nexti][1]]]
        for edge in obstacleList:
            if isEdgeIntersecting(roboedge,edge):
                return False
    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    newPoints = dict()
    adjListMap = dict()

    newPoints[1] = startPoint

    segments = []
    path = []
    g = -1

    seg1 = makeSegemnt(1,2,[startPoint, goalPoint])

    newPoints[2] = (random.uniform(0, 10), random.uniform(0, 10))

    adjListMap[1] = [2]
    adjListMap[2] = []

    seg2 = makeSegemnt(1,2,[newPoints[1], newPoints[2]])

    while RobotCollision(seg2, robot, obstacles):
        newPoints[2] = (random.uniform(0, 10),random.uniform(0, 10))
        seg2['l'] = [newPoints[1],newPoints[2]]

    segments.append(seg2)

    f = 0
    while True:

        point_x = random.uniform(0, 10)
        point_y = random.uniform(0, 10)   

        index = len(newPoints)+1
        point = (point_x, point_y)
        newPoints[index] = point
        adjListMap[index] = []

        minDist = float("inf")
        minIndex = -1
        minPoint = [-1,-1]
        isPointGenerated = False
        minp1 = None
        minp2 = None

        if isPointColliding(obstacles, (point_x,point_y)):
            continue

        count = 0
        while(count < len(segments)):         
            segment = segments[count]
            count += 1
            cI = [-100,-100]
            
            cp, newIsh, distance = ShortestDistToPoint(segment['l'][0], segment['l'][1], point)

            if np.array_equal(cp,segment['l'][0]):
                cI = segment['p1']
            elif np.array_equal(cp,segment['l'][1]):
                cI = segment['p2']

            if distance < minDist:
                minDist = distance
                minIndex = cI
                minPoint = cp
                isPointGenerated = newIsh
                minp1 = segment['p1']
                minp2 = segment['p2']

        if not isPointGenerated:

            adjListMap[minIndex].append(index)
            seg3 = makeSegemnt(minIndex, index, [newPoints[minIndex],newPoints[index]])

            if RobotCollision(seg3, robot, obstacles):
                adjListMap[minIndex].remove(index)
                continue 

            segments.append(seg3)

        else: 
            newIndex = len(newPoints)+1
            newPoints[newIndex] = minPoint
            adjListMap[newIndex] = [index]
            
            seg4 = makeSegemnt(newIndex, index, [newPoints[newIndex], newPoints[index]])
            
            if RobotCollision(seg4, robot, obstacles):
                del newPoints[newIndex]
                del adjListMap[newIndex]
                continue 
            elif minp1 == None:
                continue
            else:
                segments.append(seg4)


            if minp2 in adjListMap[minp1]:
                adjListMap[minp1].remove(minp2)

                for index, segment in enumerate(segments):
                    if segment['p1'] == minp1 and segment['p2'] == minp2:
                        del(segments[index])

                adjListMap[minp1].append(newIndex)
                adjListMap[newIndex].append(minp2)

                segments.append(makeSegemnt(minp1, newIndex, [newPoints[minp1],newPoints[newIndex]]))

                segments.append(makeSegemnt(newIndex, minp2, [newPoints[newIndex],newPoints[minp2]]))

            elif minp1 in adjListMap[minp2]:
                adjListMap[minp2].remove(minp1)
                adjListMap[minp2].append(newIndex)
                adjListMap[newIndex].append(minp1)

                for index, segment in enumerate(segments):
                    if segment['p1'] == minp2 and segment['p2'] == minp1:
                        del(segments[index])

                segments.append(makeSegemnt(minp2, newIndex, [newPoints[minp2],newPoints[newIndex]]))

                segments.append(makeSegemnt(newIndex, minp1,[newPoints[newIndex],newPoints[minp1]]))

           

        goalPointList = dict(newPoints)
        nMap = dict(adjListMap)
        nSegs = list(segments)


        index = len(goalPointList) + 1
        nMap[index] = []
        goalPointList[index] = goalPoint

        minDist = float("inf")
        minIndex = -1
        minPoint = [-1,-1]
        isPointGenerated = False
        minp1 = None
        minp2 = None
        changeMade = False
        collision = False;

        count = 0
        while(count < len(nSegs)):
            segment = nSegs[count]
            count += 1

            cp, nPoint, distance = ShortestDistToPoint(segment['l'][0], segment['l'][1], goalPoint)

            ci = [-1,-1]

            if np.array_equal(cp,segment['l'][0]):
                ci = segment['p1']
            elif np.array_equal(cp,segment['l'][1]):
                ci = segment['p2']

            if distance < minDist:
                minDist = distance
                minIndex = ci
                minPoint = cp
                isPointGenerated = nPoint
                minp1 = segment['p1']
                minp2 = segment['p2']

        if not isPointGenerated:
            nMap[minIndex].append(index)
            seg10 = makeSegemnt(minIndex, index, [goalPointList[minIndex],goalPointList[index]])

            if RobotCollision(seg10, robot, obstacles):
                nMap[minIndex].remove(index)
                changeMade = True
                connects = False

            nSegs.append(seg10)

        else: 
            newIndex = len(goalPointList)+1
            goalPointList[newIndex] = minPoint
            nMap[newIndex] = [index]

            seg9 = makeSegemnt(newIndex, index, [goalPointList[newIndex], goalPointList[index]])

            if RobotCollision(seg9, robot, obstacles):
                del nMap[newIndex]
                del goalPointList[newIndex]
                collision = True
                changeMade = True
                connects = False

            nSegs.append(seg9)

            if minp1 == None:
                changeMade = True
                connects = False

            if not minp2 in nMap[minp1] and not collision:

                if minp1 in nMap[minp2]:
                    nMap[minp2].remove(minp1)
                    nMap[minp2].append(newIndex)
                    nMap[newIndex].append(minp1)

                    for index, segment in enumerate(nSegs):
                        if segment['p1'] == minp2 and segment['p2'] == minp1:
                            del(nSegs[index])

                    nSegs.append(makeSegemnt(minp2, newIndex,[goalPointList[minp2],goalPointList[newIndex]] ))

                    seg13 = dict()
                    seg13['p1'] = newIndex
                    seg13['p2'] = minp1
                    seg13['l'] = [goalPointList[newIndex],goalPointList[minp1]]
                    nSegs.append(makeSegemnt(newIndex, minp1, [goalPointList[newIndex],goalPointList[minp1]]))

            elif not collision:
                nMap[minp1].remove(minp2)

                for index, segment in enumerate(nSegs):
                    if segment['p1'] == minp1 and segment['p2'] == minp2:
                        del(nSegs[index])


                nMap[minp1].append(newIndex)
                nMap[newIndex].append(minp2)

                nSegs.append(makeSegemnt(minp1, newIndex, [goalPointList[minp1],goalPointList[newIndex]]))

                nSegs.append(makeSegemnt(newIndex, minp2, [goalPointList[newIndex],goalPointList[minp2]]))


        if not changeMade :
            connects = True

        if not connects:
            pass;
        else:
            newPoints = goalPointList
            adjListMap = nMap
            segments = nSegs 
            break

    #for i in range(1,len(newPoints)):
    i = 1
    while i < len(newPoints):

        x = len(newPoints)+1 - i
        i+=1
        if not newPoints[x][0] == goalPoint[0] or not newPoints[x][1] == goalPoint[1]:
            continue
        else:
            g = x
            break

    return newPoints, adjListMap, basicSearch(adjListMap, 1, g)


if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print ""
    print "The input points are:"
    print str(points)
    print ""

    points, adjListMap = growSimpleRRT(points)

    # Search for a solution
    path = basicSearch(adjListMap, 1, len(points))

    # Your visualization code
    displayRRTandPath(points, adjListMap, path)

    # Solve a real RRT problem
    new_points, super_tree, the_path = RRT(robot, obstacles, (x1, y1), (x2, y2))

    # Your visualization code
    displayRRTandPath(new_points, super_tree, the_path, robotStart, robotGoal, obstacles)