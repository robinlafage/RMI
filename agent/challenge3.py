
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import time 
#TODO In script, install this lib using pip install dijkstra 
import dijkstra as dijkstraLib

CELLROWS=7
CELLCOLS=14
SPEED = 0.15
OFFSET = 0.04
NEW_CELL_THRESHOLD = 2.0
SENSOR_THRESHOLD = 1.1
ROTATION_DEG = 65


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.graph = dijkstraLib.Graph()
        self.prevPos = []
        self.beacons_positions   = {}
        self.intersections       = []
        self.start_positions     = []
        self.visiting_beacon     = []
        self.notFinishedVisiting = []
        self.previousPositions    = []
        self.visited = []
        self.path = []
        self.goingBack = False
        self.hasTurned = False
        self.followingPath = False


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

        if self.measures.ground <= 0 : 
            for index, beacon in enumerate(self.visiting_beacon) :
                if beacon == True :
                    self.visiting_beacon[index] = False

        self.beaconsManagement()
        #We need to set these 2 variables after the call to beaconsManagement because otherwise self.start_positions would not be set
        x = self.measures.x - self.start_positions[0]
        y = self.measures.y - self.start_positions[1]
        roundedPositions = self.roundPositions([x,y])

        if self.previousPositions == []:
            self.previousPositions.append(roundedPositions)
            self.prevPos = roundedPositions
            
        dir = self.measures.compass
        
        if self.followingPath == True : 
            if ((abs(roundedPositions[0] - self.prevPos[0]) >= NEW_CELL_THRESHOLD)  or (abs(roundedPositions[1] - self.prevPos[1]) >= NEW_CELL_THRESHOLD)) :
                self.prevPos = roundedPositions
            self.followThePath(self.measures.compass)

            walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)
            nextVisitedCells = self.nextVisitedCells(dir)
            finishedIntersection = self.allVisitedHere(walls, nextVisitedCells)
            if finishedIntersection and (roundedPositions[0], roundedPositions[1]) in self.intersections :
                self.intersections.remove((roundedPositions[0], roundedPositions[1]))
            elif self.isIntersection(walls) and (roundedPositions[0],roundedPositions[1]) not in self.intersections and not finishedIntersection:
                self.intersections.append((roundedPositions[0],roundedPositions[1]))
                
        
        # #Visiting a new cell
        elif ((abs(x - self.prevPos[0]) >= NEW_CELL_THRESHOLD)  or (abs(y - self.prevPos[1]) >= NEW_CELL_THRESHOLD)) :
            
            #If the position is not already in the list of visited positions, we add it
            if (not roundedPositions in self.previousPositions) :
                print(f"\n\nNEW CELL\nposition : {roundedPositions}")
                self.previousPositions.append(roundedPositions) 
            if roundedPositions not in self.visited :
                self.visited.append(roundedPositions)

            #Adding an edge to the graph
            if (self.prevPos[0],self.prevPos[1]) not in self.graph.get_adjacent_nodes((roundedPositions[0],roundedPositions[1])) :
                self.graph.add_edge((self.prevPos[0],self.prevPos[1]), (roundedPositions[0],roundedPositions[1]), 1)
                self.graph.add_edge((roundedPositions[0],roundedPositions[1]), (self.prevPos[0],self.prevPos[1]), 1)
            
            #Updating the value of prevPos
            self.prevPos = roundedPositions
            walls = self.getWalls(centerSensor, leftSensor, rightSensor, backSensor)
            nextVisitedCells = self.nextVisitedCells(dir)
            finishedIntersection = self.allVisitedHere(walls, nextVisitedCells)
            if finishedIntersection and (roundedPositions[0], roundedPositions[1]) in self.intersections :
                self.intersections.remove((roundedPositions[0], roundedPositions[1]))
            elif self.isIntersection(walls) and (roundedPositions[0],roundedPositions[1]) not in self.intersections and not finishedIntersection:
                self.intersections.append((roundedPositions[0],roundedPositions[1]))

            
            self.chooseDirection(walls, dir, roundedPositions[0], roundedPositions[1])

            # If the robot has turned, we stop the motors just one time to keep the right direction 
            if self.hasTurned:
                self.driveMotors(0.0, 0.0)
                self.hasTurned = False
        
        else:
            self.goForward(dir)
        
            
        
    #Utile ?   
    def motorsCommand(self) : 
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        if self.measures.irSensor[center_id] > 1.5 :
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id]\
                 and self.measures.irSensor[left_id] > 1 :
                self.driveMotors(0.15,-0.15)
            elif self.measures.irSensor[right_id] > self.measures.irSensor[left_id] \
                 and self.measures.irSensor[left_id] > 1:
                self.driveMotors(-0.15,0.15)
            else : 
                self.driveMotors(-0.15,0.15)
        else : 
            self.goForward(self.measures.compass)
    

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
                    self.beacons_positions[beacon] = self.averagePosition(self.beacons_positions[beacon])

        #If we are on start, we set the start_points here 
        if self.measures.ground == 0 :
            if self.start_positions == []:
                self.start_positions = [self.measures.x, self.measures.y]
                self.beacons_positions[0] = [0,0]
            print("We are on start point !")
        
        #We register the positions relatives to the start_points until we get out of the beacon 
        elif self.measures.ground > 0 : 
            print("Visiting a beacon !")
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
                self.beacons_positions[beacon].append([self.measures.x - self.start_positions[0], self.measures.y - self.start_positions[1]])
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

    
    def goForward(self, dir):
        #If we are not deviating from the direction, we keep going
        # Un degré de tolérance, surement a supprimer #if (dir >= -1 and dir <= 1) or (dir >= 89 and dir <= 91) or (dir >= 179 and dir <= 180) or (dir <= -180 and dir >= -181) or (dir >= -91 and dir <= -89):
        if dir == 0 or dir == 90 or dir == -90 or dir == 180 or dir == -180:
            self.driveMotors(SPEED, SPEED)
            return
        
        #Else, we adjust the direction

        if (dir <= 45 and dir > 0) or (dir <= 135 and dir > 90) or (dir <= -135 and dir > -180) or (dir <= -45 and dir > -90):
            #Turn slowly right
            self.driveMotors(SPEED, SPEED-OFFSET)

        elif (dir >= -45 and dir < 0) or (dir >= 45 and dir < 90) or (dir >= 135 and dir < 180 and dir > 0) or (dir >= -135 and dir < -90):
            #Turn slowly left
            self.driveMotors(SPEED-OFFSET, SPEED)

    def getWalls(self, centerSensor, leftSensor, rightSensor, backSensor):
        walls = [False, False, False, False]
        if centerSensor >= SENSOR_THRESHOLD:
            walls[0] = True
        # If we are not sure there is a wall in front of us, we keep going and check again
        elif centerSensor >= 0.8 and centerSensor < SENSOR_THRESHOLD:
            self.driveMotors(SPEED, SPEED)
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

        if not any(walls):
            print("pas de mur")

        return walls
    
     #Return a list of the adjacent cells that have already been visited
    #The list is ordered as follows: [forward, left, right, back], from the point of view of the robot
    def nextVisitedCells(self, dir):
        visitedCells = [False, False, False, False]
        x = self.measures.x - self.start_positions[0]
        y = self.measures.y - self.start_positions[1]
        roundedPositions = self.roundPositions([x,y])
        x = roundedPositions[0]
        y = roundedPositions[1]

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

        print(visitedCells)
        print(dir)
        print()
        return visitedCells
    
    #TODO Make this function viable
    def turnToDirectionTarget(self, idealDirection):
        direction = self.measures.compass
        # idealDirection2 = 0
        # if idealDirection == 180 :
        #     idealDirection2 = -180
        # while not ((direction <= idealDirection+3 and direction >= idealDirection-3) or ((idealDirection2 == -180 or idealDirection2 == -90) and direction <= idealDirection2+3 and direction >= idealDirection2-3)) :
        #     self.driveMotors(-0.05,0.05)
        #     self.readSensors()
        #     direction = self.measures.compass
        #     if direction <= -160 and direction >= -200 :
        #         direction = 360 + direction
        #     self.hasTurned = True
        
        
        if idealDirection == 0 :
            while direction <= -3 or direction >= 3 :
                if direction >= -180 and direction <= 0 :
                    self.driveMotors(-0.05,0.05)
                else :
                    self.driveMotors(0.05,-0.05)
                self.readSensors()
                direction = self.measures.compass
                print(direction)
        if idealDirection == 90 :
            while direction <= 87 or direction >= 93 :
                if direction >= -90 and direction <= 90 :
                    self.driveMotors(-0.15,0.15)
                else :
                    self.driveMotors(0.15,-0.15)
                self.readSensors()
                direction = self.measures.compass
                print(direction)
        if idealDirection == 180 :
            while (direction <= 177 and direction > 0) or (direction >= -177 and direction < 0):
                if direction >= 0 and direction <= 180 :
                    self.driveMotors(-0.15,0.15)
                else :
                    self.driveMotors(0.15,-0.15)
                self.readSensors()
                direction = self.measures.compass
                print(direction)
        if idealDirection == -90 :
            while direction <= -93 or direction >= -87 :
                if (direction >= 90 and direction <= 180) or (direction <= -90 and direction >= -180) :
                    self.driveMotors(-0.15,0.15)
                else :
                    self.driveMotors(0.15,-0.15)
                self.readSensors()
                direction = self.measures.compass
                print(direction)
        self.hasTurned = True


        self.driveMotors(0.0, 0.0)
        
        
        

    
    #TODO : Adapt this function
    # At each cell, choose the right direction to take, and start going in that direction
    def chooseDirection(self, walls, dir, x, y):
        if self.goingBack:
            pop2 = True
        else:
            pop2 = False

        nextVisitedCells = self.nextVisitedCells(dir)

        if (not walls[0] and not nextVisitedCells[0]):
            print("go forward")
            self.hasTurned = False
            self.driveMotors(SPEED, SPEED)
        elif (not walls[1] and not nextVisitedCells[1]):
            print("turn left")
            self.hasTurned = True
            #To avoid the case where the robot is facing the wall and the direction is positive
            if dir <= 180 and dir >= 170:
                dir = -180
            currentDir = dir
            while dir <= currentDir + ROTATION_DEG:
                self.driveMotors(-SPEED, SPEED)
                self.readSensors()
                dir = self.measures.compass
            self.driveMotors(-SPEED, SPEED)
        elif (not walls[2] and not nextVisitedCells[2]):
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
        else:
            print("Path entirely explored, going back to the previous intersection")
            x = self.measures.x - self.start_positions[0]
            y = self.measures.y - self.start_positions[1]
            roundedPositions = self.roundPositions([x,y])
            pathToNextIntersection = self.bestWayToGoThere((roundedPositions[0],roundedPositions[1]))
            
            self.path = pathToNextIntersection
            self.followThePath(dir)
        
        return pop2

    #Return True if the current cell is an intersection, i.e if there is more than one way to go, except the way we came from
    def isIntersection(self, walls):
        return walls[:-1].count(False) > 1

    def allVisitedHere(self, walls, nextVisitedCells):
        result = True
        for index, booleen in enumerate(nextVisitedCells) :
            if booleen == False :
                if walls[index] == False :
                    result = False
        return result

    
    def followThePath(self, dir):
        self.followingPath = True
        print(self.path)
        if len(self.path) == 1 :
            x = self.measures.x - self.start_positions[0]
            y = self.measures.y - self.start_positions[1]
            roundedPosition = self.roundPositions([x,y])
            #Add verification that we are in the cell
            if (roundedPosition[0], roundedPosition[1]) == self.path[0] :
                self.path.pop(0)
                self.followingPath = False
                print("finished following the path")
                print(roundedPosition)
        else : 
            actualPosition = self.path[0]
            nextPosition = self.path[1]
            if (actualPosition[0]+2 == nextPosition[0] and actualPosition[1] == nextPosition[1]):
                self.turnToDirectionTarget(0.0)
                self.goForward(0.0)
            elif (actualPosition[0]-2 == nextPosition[0] and actualPosition[1] == nextPosition[1]):
                self.turnToDirectionTarget(180.0)
                self.goForward(180)
            elif (actualPosition[1]+2 == nextPosition[1] and actualPosition[0] == nextPosition[0]):
                self.turnToDirectionTarget(90.0)
                self.goForward(90.0)
            elif (actualPosition[1]-2 == nextPosition[1] and actualPosition[0] == nextPosition[0]):
                self.turnToDirectionTarget(-90.0)
                self.goForward(-90.0)
            else :
                print("THE PATH IS WRONG")
                print(self.graph.get_adjacent_nodes((0,0)))
            
            #Add verification that we are in the cell
            x = self.measures.x - self.start_positions[0]
            y = self.measures.y - self.start_positions[1]
            roundedPosition = self.roundPositions([x,y])
            if (roundedPosition[0], roundedPosition[1]) == self.path[1] :
                self.path.pop(0)
        

        
    def bestWayToGoThere(self, originCell):
        distances = []
        bestDistance = 100
        dijkstra = dijkstraLib.DijkstraSPF(self.graph, originCell)
        #TODO Modify this with the intersection list
        for pos in self.intersections :
            distance = dijkstra.get_distance(pos)
            if distance < bestDistance :
                    bestPosition = pos
                    bestDistance = distance
        if self.intersections == [] :
            resultat = []
        else :
            resultat = dijkstra.get_path(bestPosition)
        return resultat




# graph={"a":["b","c","d","h"],"b":["a","e"], "c":["a","e"], "d" : ["a","f"], "e":["b","c","f","g"], "f":["d","e","h"], "g":["e", "h"], "h":["f","g","a"]}

# graph = dijkstra.Graph()
# graph.add_edge("a","b",1)
# graph.add_edge("a","c",1)
# graph.add_edge("a","d",1)
# graph.add_edge("a","h",1)
# graph.add_edge("b","e",1)
# graph.add_edge("c","e",1)
# graph.add_edge("d","f",1)
# graph.add_edge("e","f",1)
# graph.add_edge("e","g",1)
# graph.add_edge("f","h",1)
# graph.add_edge("g","h",1)

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


rob_name = "pClient3"
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
    rob=MyRob(rob_name,pos,[0.0,85.0,-85.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
