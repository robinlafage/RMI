
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, previous_distances):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.previous_distances  = previous_distances

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
        
        # These statements make the robot rotate when there is a wall in front
        if    self.measures.irSensor[center_id]  > 1.1\
            and self.measures.irSensor[right_id] > self.measures.irSensor[left_id]\
            and self.measures.irSensor[right_id] > 1.15:
            print('Rotate left')
            self.driveMotors(-0.15,+0.15)
        elif    self.measures.irSensor[center_id]  > 1.1\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]\
            and self.measures.irSensor[left_id] > 1.15:
            print('Rotate right')
            self.driveMotors(+0.15,-0.15)

        # Security statements, to avoid collision with a lateral wall
        elif self.measures.irSensor[left_id]> 15.0\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]:
            print('Rotate fast right, too close from the wall')
            self.driveMotors(0.15,-0.05)
        elif self.measures.irSensor[right_id]> 15.0\
            and self.measures.irSensor[right_id]   > self.measures.irSensor[left_id]:
            print('Rotate fast left, too close from the wall')
            self.driveMotors(-0.05,0.15)
        
        # Security statements, to avoid collision with a front wall
        elif self.measures.irSensor[center_id]> 15.0\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]:
            print('Rotate fast right, too close from the wall')
            self.driveMotors(0.15,-0.15)
        elif self.measures.irSensor[center_id]> 15.0\
            and self.measures.irSensor[right_id]   > self.measures.irSensor[left_id]:
            print('Rotate fast left, too close from the wall')
            self.driveMotors(-0.15,0.15)

        # These two statements make the robot go as straight as possible
        elif self.previous_distances\
            and self.previous_distances[left_id] > self.measures.irSensor[left_id]\
            and self.previous_distances[right_id] < self.measures.irSensor[right_id]:
            print('Navigate to the left to center the robot')
            self.driveMotors(0.12,0.15)
        elif self.previous_distances\
            and self.previous_distances[left_id] < self.measures.irSensor[left_id]\
            and self.previous_distances[right_id] > self.measures.irSensor[right_id]:
            print('Navigate to the right to center the robot')
            self.driveMotors(0.15,0.12)

        else:
            print('Go')
            self.driveMotors(0.15,0.15)
        
        # Save the previous distances in order to permit to make the robot go straight
        self.previous_distances=[self.measures.irSensor[center_id],self.measures.irSensor[left_id],self.measures.irSensor[right_id],self.measures.irSensor[back_id]]

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


rob_name = "pClient1"
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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host, [])
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
