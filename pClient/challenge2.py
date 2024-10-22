
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import time
import os

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
        self.prevPos = prevPos
        self.intersections = intersections
        self.visited = visited
        self.goingBack = False
        self.hasTurned = False
        self.outputFile = open("map.map", "w")

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

        self.initMap()

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
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        centerSensor = self.measures.irSensor[center_id]
        leftSensor = self.measures.irSensor[left_id]
        rightSensor = self.measures.irSensor[right_id]
        backSensor = self.measures.irSensor[back_id]

        x = roundTo05(self.measures.x)
        y = roundTo05(self.measures.y)
        dir = self.measures.compass
        
        #We arrive in a new cell
        try:
            if not self.prevPos or abs(x - self.prevPos[-1][0]+2) <= NEW_CELL_THRESHOLD or abs(y - self.prevPos[-1][1]+2) <= NEW_CELL_THRESHOLD or abs(x - self.prevPos[-1][0]-2) <= NEW_CELL_THRESHOLD or abs(y - self.prevPos[-1][1]-2) <= NEW_CELL_THRESHOLD:
                print()
                print("-----------------------------")
                print("New cell\n")
                if not self.goingBack:
                    self.prevPos.append([x, y])
                self.visited.append([round(x), round(y)])
                walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)
                self.writeMap(walls, dir)
                print()
                if self.isIntersection(walls) and [round(x), round(y)] not in self.intersections:
                    self.intersections.append([round(x), round(y)])

                pop = self.chooseDirection(walls, dir, x, y, True)

                if self.goingBack or pop:
                    self.prevPos.pop()

                # print(self.goingBack)
                # print(self.prevPos)
                
                if self.hasTurned:
                    self.driveMotors(0.0, 0.0)
                # time.sleep(1)

            #If we are not in a new cell, we keep going
            else:
                self.goForward(dir)
        except:
            self.goingBack = False
            nextVisitedCells = self.nextVisitedCells(dir)
            walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)
            if (not walls[0] and not nextVisitedCells[0]) or (not walls[1] and not nextVisitedCells[1]) or (not walls[2] and not nextVisitedCells[2]) or (not walls[3] and not nextVisitedCells[3]):
                self.prevPos.append([x, y])
                self.wander()
            else:
                self.endChallenge()


    def getWalls(self, centerSensor, leftSensor, rightSensor, backSensor):
        walls = [False, False, False, False]
        print("centerSensor: ", centerSensor, ", leftSensor: ", leftSensor, ", rightSensor: ", rightSensor, ", backSensor: ", backSensor)
        if centerSensor >= SENSOR_THRESHOLD:
            print("mur devant")
            walls[0] = True
        elif centerSensor >= 0.8 and centerSensor < SENSOR_THRESHOLD:
            print("\033[91m" + "BELEK ON SAIT PAS TROP LA")
            self.driveMotors(SPEED, SPEED)
            time.sleep(0.015)
            self.readSensors()
            centerSensor = self.measures.irSensor[0]
            print("centerSensor: ", centerSensor, "\033[0m")
            if centerSensor >= 1.1:
                print("mur devant")
                walls[0] = True
        if leftSensor >= SENSOR_THRESHOLD:
            print("mur à gauche")
            walls[1] = True
        if rightSensor >= SENSOR_THRESHOLD:
            print("mur à droite")
            walls[2] = True
        if backSensor >= SENSOR_THRESHOLD:
            print("mur derrière")
            walls[3] = True

        if not any(walls):
            print("pas de mur")

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
            #Turn slowly right
            # print("Adjusting direction - turning right")
            self.driveMotors(SPEED, SPEED-OFFSET)

        elif (dir >= -45 and dir < 0) or (dir >= 45 and dir < 90) or (dir >= 135 and dir < 180 and dir > 0) or (dir >= -135 and dir < -90):
            #Turn slowly left
            # print("Adjusting direction - turning left")
            self.driveMotors(SPEED-OFFSET, SPEED)

    def chooseDirection(self, walls, dir, x, y, flag):
        if self.goingBack:
            pop2 = True
        else:
            pop2 = False

        nextVisitedCells = self.nextVisitedCells(dir)
        target = -1
        tmp = self.goingBack

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
            print("go forward")
            self.hasTurned = False
            self.driveMotors(SPEED, SPEED)
        elif (not walls[1] and not nextVisitedCells[1] and target == -1) or target == 1:
            print("turn left")
            self.hasTurned = True
            #To avoid the case where the robot is facing the wall and the direction is positive
            if dir <= 180 and dir >= 170:
                dir = -180
            currentDir = dir
            while dir <= currentDir + ROTATION_DEG: #à modifier en fonction de la vitesse de déplacement du robot
                self.driveMotors(-SPEED, SPEED)
                self.readSensors()
                dir = self.measures.compass
            self.driveMotors(-SPEED, SPEED)
        elif (not walls[2] and not nextVisitedCells[2] and target == -1) or target == 2:
            print("turn right")
            self.hasTurned = True
            #To avoid the case where the robot is facing the wall and the direction is degative
            if dir >= -180 and dir <= -170:
                dir = 180
            currentDir = dir
            while dir >= currentDir - ROTATION_DEG:
                self.driveMotors(SPEED, -SPEED)
                self.readSensors()
                dir = self.measures.compass
            self.driveMotors(SPEED, -SPEED)
        elif (not walls[3] and not nextVisitedCells[3] and target == -1) or target == 3:
            #A TESTER
            print("turn back")
            self.hasTurned = True
            currentDir = dir
            while True:
                self.driveMotors(SPEED, -SPEED)
                self.readSensors()
                dir = self.measures.compass
                diff = dir - currentDir
                if diff > 180:
                    diff -= 360
                elif diff < -180:
                    diff += 360

                if abs(diff) >= 160:
                    break

                # print("currentDir : ", currentDir, ", dir : ", dir, ", final : ", abs(diff))
            self.driveMotors(SPEED, -SPEED)
        else:
            print("Path entirely explored, going back to the previous intersection")
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

        print("target: ", target)
        return target

    #Return a list of the adjacent cells that have already been visited
    #The list is ordered as follows: [forward, left, right, back], from the point of view of the robot
    def nextVisitedCells(self, dir):
        visitedCells = [False, False, False, False]
        # x = round(self.visited[-1][0])
        # y = round(self.visited[-1][1])

        x = self.visited[-1][0]
        y = self.visited[-1][1]

        # roundedVisited = [[round(nb) for nb in subList] for subList in self.visited]

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

        # print([x, y])
        print(visitedCells)
        print()
        return visitedCells


    def initMap(self):
        for _ in range(27):
            self.outputFile.write(" " * 55 + "\n")

        position = (13 * (55 + 1)) + 27
        self.outputFile.seek(position)
        self.outputFile.write("I")
        self.outputFile.seek(position)
    
    def writeMap(self, walls, dir):
        currentPos = self.outputFile.tell()

        try:
            x1 = self.visited[-1][0]
            y1 = self.visited[-1][1]
            x2 = self.visited[-2][0]
            y2 = self.visited[-2][1]

            if x1 == x2 + 2:
                self.outputFile.seek(currentPos + 2)
            elif x1 == x2 - 2:
                self.outputFile.seek(currentPos - 2)
            elif y1 == y2 + 2:
                self.outputFile.seek(currentPos - 56*2)
            elif y1 == y2 - 2:
                self.outputFile.seek(currentPos + 56*2)

            currentPos = self.outputFile.tell()
            self.outputFile.write("X")
        except:
            pass

        if dir >= -10 and dir <= 10:
            if walls[0]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos - 56)

            if walls[1]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos + 56)

            if walls[2]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")


        elif dir >= 80 and dir <= 100:
            if walls[2]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos - 56)

            if walls[0]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos - 1)

            if walls[1]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

        elif dir <= -80 and dir >= -100:
            if walls[1]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos + 56)

            if walls[0]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos - 1)

            if walls[2]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

        elif dir >= 170 or dir <= -170:
            self.outputFile.seek(currentPos - 1)

            if walls[0]:
                self.outputFile.write("|")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos + 56)

            if walls[1]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")

            self.outputFile.seek(currentPos - 56)

            if walls[2]:
                self.outputFile.write("-")
            else:
                self.outputFile.write("X")

        else:
            print("\033[91m" + "INVALID DIRECTION" + "\033[0m")


        self.outputFile.seek(currentPos)


    def endChallenge(self):
        position = (13 * (55 + 1)) + 27
        self.outputFile.seek(position)
        self.outputFile.write("I")
        self.outputFile.close()
        self.finish()
        print("\033[91m")
        os.system("gawk -f ../simulator/mapping_score.awk ../simulator/mapping.out map.map")
        print("\033[0m")

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


"""
Tâches primaires :
    Stabiliser le programme au maximum (détéction des murs)
    Commenter le code
    Simplifier certaines parties du code
    Enlever les print de debug


Tâches secondaires :
    Si il existe un chemin plus court pour revenir à l'intersection précédente, le prendre
    Faire reculer le robot au lieu de le faire pivoter quand il doit faire demi-tour
"""