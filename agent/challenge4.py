import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import time
import os
import dijkstra as dijkstraLib
import maths

CELLROWS=7
CELLCOLS=14
SPEED = 0.15
OFFSET = 0.06
ROTATION_DEG = 65
NEW_CELL_THRESHOLD = 0.0
SENSOR_THRESHOLD = 1.1

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, prevPos, intersections, visited):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.graph = dijkstraLib.Graph()
        self.prevPos = prevPos
        self.intersections = intersections
        self.visited = visited
        self.visited2 = []
        self.goingBack = False
        self.hasTurned = False
        self.beacons_positions   = {}
        self.visiting_beacon     = []
        self.start_positions     = []
        self.path = []

        self.x  = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.outr  = 0.0
        self.outl = 0.0
        self.drove_left = 0.0
        self.drove_right = 0.0
        self.startValues={}
        self.t = 0
        self.thetaCoefficient = 0.9


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()

    def driveMotors(self, lPow, rPow):
        self.drove_left = lPow
        self.drove_right = rPow
        if self.thetaCoefficient > 0.5 :
            self.thetaCoefficient -= 0.01
        self.theta = self.calculateOrientation()
        self.calculatePosition()
        # print(f"X calculé : {round(self.x, 1)}, X mesuré : {round(self.measures.x - self.startValues['x'], 2)}")
        # print(f"Y calculé : {round(self.y, 1)}, Y mesuré : {round(self.measures.y - self.startValues['y'], 2)}")
        # print(f"Theta calculé : {round(degrees(self.theta), 1)}, Theta mesuré : {round(self.measures.compass, 2)}")
        return super().driveMotors(lPow, rPow)
    
    def calculatePosition(self):
        self.outl = maths.out(self.drove_left, self.outl)
        self.outr = maths.out(self.drove_right, self.outr)
        l=maths.lin(self.outl, self.outr)
        self.x = maths.x(self.x, l, self.theta)
        self.y = maths.y(self.y, l, self.theta)
        r=maths.rot(self.outl, self.outr, 1)
        self.theta = maths.theta(self.theta, r)

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        centerSensor = self.measures.irSensor[center_id]
        leftSensor = self.measures.irSensor[left_id]
        rightSensor = self.measures.irSensor[right_id]
        backSensor = self.measures.irSensor[back_id]

        if self.measures.ground <= 0 : 
            for index, beacon in enumerate(self.visiting_beacon) :
                if beacon == True :
                    self.visiting_beacon[index] = False

        self.beaconsManagement()

        x2 = self.x - self.start_positions[0]
        y2 = self.y - self.start_positions[1]
        roundedPositions = self.roundPositions([x2,y2])

        if self.startValues == {}:
            self.startValues['x']=0
            self.startValues['y']=0
        
        x = roundTo05(self.x)
        y = roundTo05(self.y)
        dir = round(degrees(self.theta),0)
        

        # print(f"Position: {x}, {y}")

        #We arrive in a new cell
        try:
            if not self.prevPos or abs(x - self.prevPos[-1][0]+2) <= NEW_CELL_THRESHOLD or abs(y - self.prevPos[-1][1]+2) <= NEW_CELL_THRESHOLD or abs(x - self.prevPos[-1][0]-2) <= NEW_CELL_THRESHOLD or abs(y - self.prevPos[-1][1]-2) <= NEW_CELL_THRESHOLD:
                print("New cell")
                print(f'Exact position : {self.x}')
                print(f"Position: {x}, {y}")

                if not self.goingBack:
                    self.prevPos.append([x, y])
                self.visited.append([round(x), round(y)])
                self.visited2.append(roundedPositions)

                walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)

                if walls[0]:
                    self.calculatePosWithWall(centerSensor, dir)
                
                self.addToGraph(walls,dir,roundedPositions[0],roundedPositions[1])
                
                if self.isIntersection(walls) and [round(x), round(y)] not in self.intersections:
                    self.intersections.append([round(x), round(y)])

                pop = self.chooseDirection(walls, dir, x, y, True)

                if self.goingBack or pop:
                    self.prevPos.pop()

                # If the robot has turned, we stop the motors just one time to keep the right direction 
                if self.hasTurned:
                    self.readSensors()
                    self.driveMotors(0.0, 0.0)

            #If we are not in a new cell, we keep going
            else:
                self.goForward(dir)
        except:
            # The robot arrive in the starting cell
            self.goingBack = False
            nextVisitedCells = self.nextVisitedCells(dir)
            walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)
            # If parts of the lab are not explored, we keep going
            if (not walls[0] and not nextVisitedCells[0]) or (not walls[1] and not nextVisitedCells[1]) or (not walls[2] and not nextVisitedCells[2]) or (not walls[3] and not nextVisitedCells[3]):
                self.prevPos.append([x, y])
                self.wander()
            # Else, we can stop the challenge
            else:
                #Convert dict to list :
                listOfBeacons = []
                for beacon in self.beacons_positions :
                    listOfBeacons.append(self.beacons_positions[beacon])
                for beacon in listOfBeacons :
                    startBeacon = beacon 
                    listOfBeacons.remove(beacon)
                    bestWay = self.bestWayToGoThere(beacon,listOfBeacons)
                    for i in bestWay[0] :
                        self.path.append(i)
                    break
                while (listOfBeacons != []) and (listOfBeacons != [None]) and (listOfBeacons != None):
                    listOfBeacons.remove(bestWay[1])
                    if (listOfBeacons != []) and (listOfBeacons != [None]) and (listOfBeacons != None):
                        bestWay = self.bestWayToGoThere(bestWay[1],listOfBeacons)
                        for index, position in enumerate(bestWay[0]) :
                            if index != 0 :
                                self.path.append(position)

                listOfBeacons = [startBeacon]
                bestWay = self.bestWayToGoThere(bestWay[1],listOfBeacons)
                for index, position in enumerate(bestWay[0]) :
                    if index != 0 :
                        self.path.append(position)
                self.endChallenge()

    def calculatePosWithWall(self, centerSensor, dir):
        # Go right
        xmur=None
        ymur=None
        # print()
        if abs(dir) <= 10:
            xmur = self.nextOdd(self.x)
            xrobot = xmur - 1/centerSensor - 0.6
            print("\033[31mx calculé avec le mur : ", round(xrobot, 1), "\033[0m")
            print(round(self.x - self.startValues['x'],1))
        #Go up
        elif abs(dir - 90) <= 10:
            ymur = self.nextOdd(self.y)
            yrobot = ymur - 1/centerSensor - 0.6
            print("\033[31my calculé avec le mur : ", round(yrobot, 1), "\033[0m")
            print(round(self.y - self.startValues['y'],1))
        #Go down
        elif abs(dir + 90) <= 10:
            ymur = self.previousOdd(self.y)
            yrobot = ymur + 1/centerSensor + 0.6
            print("\033[31my calculé avec le mur : ", round(yrobot, 1), "\033[0m")
            print(round(self.y - self.startValues['y'],1))
        #Go left
        elif abs(dir - 180) <= 10 or abs(dir + 180) <= 10:
            xmur = self.previousOdd(self.x)
            xrobot = xmur + 1/centerSensor + 0.6
            print("\033[31mx calculé avec le mur : ", round(xrobot, 1), "\033[0m")
            print(round(self.x - self.startValues['x'],1))

        print(f"xmur : {xmur}, ymur : {ymur}")
        
    # TODO : Sur la ligne droite en abs, on a tendance à se décaler légèrement vers la gauche, c'est bien relou
    def calculateOrientation(self):
        compassCoefficient = 1 - self.thetaCoefficient
        compass = maths.degToRad(self.measures.compass)
        # print(f'Value of compass : {compass}')
        # print(f'Value of theta : {self.theta}')
        average = self.thetaCoefficient*self.theta + compassCoefficient*compass
        if (compass > 0 and self.theta < 0) or (compass < 0 and self.theta > 0) :
            if compass > 2.9 and self.theta < -2.9 :
                compass = compass - 2*pi
                average = maths.frameAngle(self.thetaCoefficient*(self.theta) + compassCoefficient*compass)
                print(f'Average : {average}')
            elif self.theta > 2.9 and compass < -2.9 :
                compass = compass + 2*pi
                average = maths.frameAngle(self.thetaCoefficient*(self.theta) + compassCoefficient*compass)
                print(f'Average : {average}')

        return average

    def nextOdd(self, number):
        base = int(number)
        
        if base % 2 == 0:  
            return base + 1
        else: 
            return base + 2
        
    def previousOdd(self, number):
        int_number = int(number)
        if int_number >= 0:
            if int_number % 2 != 0:
                return int_number

            return int_number - 1
        else:
            if int_number % 2 != 0:
                return int_number - 2

            return int_number - 1

    def bestWayToGoThere(self, originCell,listOfBeacons):
        bestDistance = 10000
        dijkstra = dijkstraLib.DijkstraSPF(self.graph, originCell)
        for position in listOfBeacons :
            distance = dijkstra.get_distance(position)
            if distance < bestDistance :
                    bestPosition = position
                    bestDistance = distance
        if listOfBeacons == [] or listOfBeacons == [None] or listOfBeacons == None:
            resultat = []
            bestPosition = None
        else :
            resultat = dijkstra.get_path(bestPosition)
        return (resultat, bestPosition)
    
    def addToGraph(self, walls, dir,x,y):
        #For the wall in front

        if not walls[0] : 
            if dir <=45 and dir >= -45 :
                if [x+2, y] in self.visited2 :
                    self.graph.add_edge((round(x+2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x+2),round(y)), 1)
            elif dir >=45 and dir <= 135 :
                if [x, y+2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y+2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y+2)), 1)
            elif (dir >=135 and dir <=180) or (dir <= -135 and dir >= -180) :
                if [x-2, y] in self.visited2 :
                    self.graph.add_edge((round(x-2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x-2),round(y)), 1)
            elif dir <= -45 and dir >= -135 :
                if [x, y-2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y-2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y-2)), 1)
            
        #For the wall on left
        if not walls[1] : 
            if dir <=45 and dir >= -45 :
                if [x, y+2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y+2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y+2)), 1)
            elif dir >=45 and dir <= 135 :
                if [x-2, y] in self.visited2 :
                    self.graph.add_edge((round(x-2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x-2),round(y)), 1)
            elif (dir >=135 and dir <=180) or (dir <= -135 and dir >= -180) :
                if [x, y-2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y-2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y-2)), 1)
            elif dir <= -45 and dir >= -135 :
                if [x+2, y] in self.visited2 :
                    self.graph.add_edge((round(x+2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x+2),round(y)), 1)
            


        #For the wall on right
        if not walls[2] : 
            if dir <=45 and dir >= -45 :
                if [x, y-2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y-2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y-2)), 1)
            elif dir >=45 and dir <= 135 :
                if [x+2, y] in self.visited2 :
                    self.graph.add_edge((round(x+2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x+2),round(y)), 1)
            elif (dir >=135 and dir <=180) or (dir <= -135 and dir >= -180) :
                if [x, y+2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y+2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y+2)), 1)
            elif dir <= -45 and dir >= -135 :
                if [x-2, y] in self.visited2 :
                    self.graph.add_edge((round(x-2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x-2),round(y)), 1)
            


        #For the wall on the back
        if not walls[3] : 
            if dir <=45 and dir >= -45 :
                if [x-2, y] in self.visited2 :
                    self.graph.add_edge((round(x-2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x-2),round(y)), 1)
            elif dir >=45 and dir <= 135 :
                if [x, y-2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y-2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y-2)), 1)
            elif (dir >=135 and dir <=180) or (dir <= -135 and dir >= -180) :
                if [x+2, y] in self.visited2 :
                    self.graph.add_edge((round(x+2),round(y)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x+2),round(y)), 1)
            elif dir <= -45 and dir >= -135 :
                if [x, y+2] in self.visited2 :
                    self.graph.add_edge((round(x),round(y+2)), (round(x),round(y)), 1)
                    self.graph.add_edge((round(x),round(y)), (round(x),round(y+2)), 1)
            

    def beaconsManagement(self):
        #Initializing the beacons_position dict
        if self.beacons_positions == {}:
            beacon=0
            while beacon < int(self.nBeacons) :
                self.beacons_positions[beacon] = []
                self.visiting_beacon.append(False)
                beacon += 1

        #Registering beacons in beacons_positions
        beacon = 0 
        for beacon in range(int(self.nBeacons)) :
            if self.visiting_beacon[beacon]==False and self.beacons_positions[beacon] != [] :
                if isinstance(self.beacons_positions[beacon][0], list) : 
                    averagePosition = self.averagePosition(self.beacons_positions[beacon])
                    self.beacons_positions[beacon] = (averagePosition[0], averagePosition[1])
                
        #If we are on start, we set the start_points here 
        if self.measures.ground == 0 :
            if self.start_positions == []:
                self.start_positions = [self.x, self.y]
                self.beacons_positions[0] = (0,0)
        
        #We register the positions relatives to the start_points until we get out of the beacon 
        elif self.measures.ground > 0 : 
            beacon = self.measures.ground
            self.registerBeaconPositions(beacon)
            self.visiting_beacon[beacon] = True


    def registerBeaconPositions(self, beacon):
        alreadyWriten=True
        try :
            if isinstance(self.beacons_positions[beacon][0], list):
                alreadyWriten=False
        except : 
            alreadyWriten = False
        if alreadyWriten == False :
            if not self.start_positions == []:
                self.beacons_positions[beacon].append([self.x - self.start_positions[0], self.y - self.start_positions[1]])
            else : 
                print("FIRST BEACON NOT SET, PLEASE START AT FIRST BEACON")   


    def averagePosition(self, positions_list):
        xTotal = 0
        yTotal = 0
        index = 0
        for position in positions_list :
            xTotal += position[0]
            yTotal += position[1]
            index += 1
        return self.roundPositions([xTotal / index, yTotal / index])

    def roundPositions(self, positions_coordinates) :
        return_list = []
        for number in positions_coordinates :
            if int(abs(number)+0.5) > int(number) :
                if number > 0 :
                    return_list.append(int(number+0.5))
                else : 
                    return_list.append(int(number-0.5))
            else : 
                return_list.append(int(number))
        return return_list

    # Return the walls detected by the sensors
    # The list is ordered as follows: [front, left, right, back]
    def getWalls(self, centerSensor, leftSensor, rightSensor, backSensor):
        print(f'Center : {centerSensor}')
        print(f'Left : {leftSensor}')
        print(f'Right : {rightSensor}')
        print(f'Back : {backSensor}')
        walls = [False, False, False, False]
        if centerSensor >= SENSOR_THRESHOLD:
            walls[0] = True
        # If we are not sure there is a wall in front of us, we keep going and check again
        elif centerSensor >= 0.8 and centerSensor < SENSOR_THRESHOLD:
            print('HEEERE')
            self.driveMotors(0.05, 0.05)
            time.sleep(0.015)
            self.readSensors()
            centerSensor = self.measures.irSensor[0]
            if centerSensor >= 1.1:
                walls[0] = True
        if leftSensor >= SENSOR_THRESHOLD:
            walls[1] = True
        if rightSensor >= SENSOR_THRESHOLD:
            walls[2] = True
        if backSensor >= SENSOR_THRESHOLD:
            walls[3] = True

        return walls
    
    #Return True if the current cell is an intersection, i.e if there is more than one way to go, except the way we came from
    def isIntersection(self, walls):
        return walls[:-1].count(False) > 1
    
    def goForward(self, dir):
        #If we are not deviating from the direction, we keep going
        if dir == 0 or dir == 90 or dir == -90 or dir == 180 or dir == -180:
            self.driveMotors(SPEED, SPEED)
            return
        
        #Else, we adjust the direction

        if (dir <= 45 and dir > 0) or (dir <= 135 and dir > 90) or (dir <= -135 and dir > -180) or (dir <= -45 and dir > -90):
            self.driveMotors(SPEED, SPEED-OFFSET)

        elif (dir >= -45 and dir < 0) or (dir >= 45 and dir < 90) or (dir >= 135 and dir < 180 and dir > 0) or (dir >= -135 and dir < -90):
            self.driveMotors(SPEED-OFFSET, SPEED)

    # At each cell, choose the right direction to take, and start going in that direction
    def chooseDirection(self, walls, dir, x, y, flag):
        if self.goingBack:
            pop2 = True
        else:
            pop2 = False

        nextVisitedCells = self.nextVisitedCells(dir)
        target = -1
        tmp = self.goingBack
        print(f'Walls : {walls}')

        if [round(x), round(y)] in self.intersections:
            pop = True
            for i in range(4):
                if not nextVisitedCells[i] and not walls[i]:
                    pop = False
                    break
            if pop:
                self.intersections.pop()

        if [round(x), round(y)] in self.intersections and flag:
            self.goingBack = False

        if self.goingBack:
            target = self.getDirectionTarget(dir)
            nextVisitedCells = [False, False, False, False]
        
            

        if (not walls[0] and not nextVisitedCells[0] and target == -1) or target == 0:
            self.hasTurned = False
            self.driveMotors(SPEED, SPEED)
        elif (not walls[1] and not nextVisitedCells[1] and target == -1) or target == 1:
            self.hasTurned = True
            #To avoid the case where the robot is facing the wall and the direction is positive
            if dir <= 180 and dir >= 170:
                dir = -180
            currentDir = dir
            while dir <= currentDir + ROTATION_DEG:
                self.driveMotors(-SPEED, SPEED)
                self.readSensors()
                dir = round(degrees(self.theta),0)
            self.driveMotors(-SPEED, SPEED)
        elif (not walls[2] and not nextVisitedCells[2] and target == -1) or target == 2:
            self.hasTurned = True
            #To avoid the case where the robot is facing the wall and the direction is degative
            if dir >= -180 and dir <= -170:
                dir = 180
            currentDir = dir
            while dir >= currentDir - ROTATION_DEG:
                self.driveMotors(SPEED, -SPEED)
                self.readSensors()
                dir = round(degrees(self.theta),0)
            self.driveMotors(SPEED, -SPEED)
        elif (not walls[3] and not nextVisitedCells[3] and target == -1) or target == 3:
            print('demi tour')
            self.hasTurned = True
            currentDir = dir
            while True:
                self.driveMotors(SPEED, -SPEED)
                self.readSensors()
                dir = round(degrees(self.theta),0)
                diff = dir - currentDir
                if diff > 180:
                    diff -= 360
                elif diff < -180:
                    diff += 360

                if abs(diff) >= 160:
                    break

            self.driveMotors(SPEED, -SPEED)
        else:
            print('demi-tour else')
            if tmp:
                self.prevPos.pop()
                self.prevPos.append([x, y])
            self.goingBack = True
            self.prevPos.append([x, y]) 
            self.chooseDirection(walls, dir, x, y, False)
        
        return pop2
            
    
    #Return the direction to take to go back to the previous intersection
    #0: forward, 1: left, 2: right, 3: back
    def getDirectionTarget(self, dir):
        target = -1
        diffX = round(self.prevPos[-2][0] - self.prevPos[-3][0])
        diffY = round(self.prevPos[-2][1] - self.prevPos[-3][1])
        if abs(dir) <= 10:
            if diffX > 0:
                target = 3
            elif diffX < 0:
                target = 0
            elif diffY > 0:
                target = 2
            elif diffY < 0:
                target = 1
        elif abs(dir - 90) <= 10:
            if diffX > 0:
                target = 1
            elif diffX < 0:
                target = 2
            elif diffY > 0:
                target = 3
            elif diffY < 0:
                target = 0
        elif abs(dir + 90) <= 10:
            if diffX > 0:
                target = 2
            elif diffX < 0:
                target = 1
            elif diffY > 0:
                target = 0
            elif diffY < 0:
                target = 3
        elif abs(dir - 180) <= 10 or abs(dir + 180) <= 10:
            if diffX > 0:
                target = 0
            elif diffX < 0:
                target = 3
            elif diffY > 0:
                target = 1
            elif diffY < 0:
                target = 2

        return target

    #Return a list of the adjacent cells that have already been visited
    #The list is ordered as follows: [forward, left, right, back], from the point of view of the robot
    def nextVisitedCells(self, dir):
        visitedCells = [False, False, False, False]
        x = self.visited[-1][0]
        y = self.visited[-1][1]

        if abs(dir) <= 10:
            visitedCells[0] = [x+2, y] in self.visited
            visitedCells[1] = [x, y+2] in self.visited
            visitedCells[2] = [x, y-2] in self.visited
            visitedCells[3] = [x-2, y] in self.visited
        elif abs(dir - 90) <= 10:
            visitedCells[0] = [x, y+2] in self.visited
            visitedCells[1] = [x-2, y] in self.visited
            visitedCells[2] = [x+2, y] in self.visited
            visitedCells[3] = [x, y-2] in self.visited
        elif abs(dir + 90) <= 10:
            visitedCells[0] = [x, y-2] in self.visited
            visitedCells[1] = [x+2, y] in self.visited
            visitedCells[2] = [x-2, y] in self.visited
            visitedCells[3] = [x, y+2] in self.visited
        elif abs(dir - 180) <= 10 or abs(dir + 180) <= 10:
            visitedCells[0] = [x-2, y] in self.visited
            visitedCells[1] = [x, y-2] in self.visited
            visitedCells[2] = [x, y+2] in self.visited
            visitedCells[3] = [x+2, y] in self.visited

        return visitedCells
    


    # End the challenge by closing the output file, printing the score, rewrite the starting position if needed and exit
    def endChallenge(self):
        stringToWrite = ""
        for index, position in enumerate(self.path) :
            if index != 0 :
                stringToWrite = stringToWrite + "\n"
            stringToWrite = stringToWrite + f"{position[0]} {position[1]}"
        with open("planning.path","w") as file :
            file.write(stringToWrite)
            file.close
        self.finish()

# Round a number to the nearest 0.5
def roundTo05(x):
    return round(x*2)/2

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient2"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,85.0,-85.0,180.0],host, list(), list(), list())
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
