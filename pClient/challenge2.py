
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14
SPEED = 0.1

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, prevPos, intersections, visited):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.prevPos = prevPos
        self.intersections = intersections
        self.visited = visited

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
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        centerSensor = self.measures.irSensor[center_id]
        leftSensor = self.measures.irSensor[left_id]
        rightSensor = self.measures.irSensor[right_id]
        backSensor = self.measures.irSensor[back_id]

        x = self.measures.x
        y = self.measures.y
        dir = self.measures.compass
        
        #We arrive in a new cell
        if not self.prevPos or x == self.prevPos[-1][0] + 2 or y == self.prevPos[-1][1] + 2 or x == self.prevPos[-1][0] - 2 or y == self.prevPos[-1][1] - 2:
            print()
            self.prevPos.append([x, y])
            self.visited.append([x, y])
            walls = self.getWalls(dir, centerSensor, leftSensor, rightSensor, backSensor)
            if self.isIntersection(walls):
                self.intersections.append([x, y])

            for i in range(len(walls)):
                if not walls[i]:
                    if i == 0:
                        #go forward
                        print("go forward")
                        self.driveMotors(SPEED, SPEED)
                        break
                    elif i == 1:
                        #turn left
                        print("turn left")
                        #To avoid the case where the robot is facing the wall and the direction is positive
                        if dir <= 180 and dir >= 170:
                            dir = -180
                        currentDir = dir
                        while dir <= currentDir + 65: #à modifier en fonction de la vitesse de déplacement du robot
                            self.driveMotors(-SPEED, SPEED)
                            self.readSensors()
                            dir = self.measures.compass
                        self.driveMotors(-SPEED, SPEED)
                        break
                    elif i == 2:
                        #turn right
                        print("turn right")
                        #To avoid the case where the robot is facing the wall and the direction is degative
                        if dir >= -180 and dir <= -170:
                            dir = 180
                        currentDir = dir
                        while dir >= currentDir - 65:
                            self.driveMotors(SPEED, -SPEED)
                            self.readSensors()
                            dir = self.measures.compass
                        self.driveMotors(SPEED, -SPEED)
                        break
                    elif i == 3:
                        #turn back A TESTER
                        print("turn back")
                        currentDir = dir
                        while dir <= currentDir + 180:
                            self.driveMotors(-SPEED, SPEED)
                            self.readSensors()
                            dir = self.measures.compass
                        self.driveMotors(-SPEED, SPEED)
                        break

        #If we are not in a new cell, we keep going
        else:
            self.goForward(dir)


    def getWalls(self, dir, centerSensor, leftSensor, rightSensor, backSensor):
        walls = [False, False, False, False]
        if centerSensor >= 1.3:
            print("mur devant")
            walls[0] = True
        if leftSensor >= 1.3:
            print("mur à gauche")
            walls[1] = True
        if rightSensor >= 1.3:
            print("mur à droite")
            walls[2] = True
        if backSensor >= 1.3:
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
        if (dir >= -1 and dir <= 1) or (dir >= 89 and dir <= 91) or (dir >= 179 and dir <= 180) or (dir <= -180 and dir >= -181) or (dir >= -91 and dir <= -89):
            self.driveMotors(SPEED, SPEED)
            return
        
        #Else, we adjust the direction

        if (dir <= 45 and dir > 0) or (dir <= 135 and dir > 90) or (dir <= -135 and dir > -180) or (dir <= -45 and dir > -90):
            #Turn slowly right
            print("Adjusting direction - turning right")
            self.driveMotors(SPEED, SPEED-0.03)

        if (dir >= -45 and dir < 0) or (dir >= 45 and dir < 90) or (dir >= 135 and dir < 180 and dir > 0) or (dir >= -135 and dir < -90):
            #Turn slowly left
            print("Adjusting direction - turning left")
            self.driveMotors(SPEED-0.03, SPEED)


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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host, list(), list(), list())
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()


"""
Une stack avec les positions précédentes, qui se dépile quand on reviens sur nos pas
Une stack avec les intersections, on empile à chaque nouvelle, et on dépile quand on l'a entièrement explorée
Une liste avec toutes les positions, pour ne pas repasser dessus
Ajuster la direction à prendre en fonction de l'orientation du robot, pour toujours rouler droit
"""