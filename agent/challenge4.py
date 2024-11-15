import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from maths import *

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.x  = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.outr  = 0.0
        self.outl = 0.0
        self.drove_left = 0.0
        self.drove_right = 0.0
        self.startValues={}

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
        
        drive_left = 0
        drive_right = 0

        if self.startValues == {}:
            self.startValues['x']=self.measures.x
            self.startValues['y']=self.measures.y
        
        self.outl = out(self.drove_left, self.outl)
        self.outr = out(self.drove_right, self.outr)
        l=lin(self.outl, self.outr)
        self.x = x(self.x, l, self.theta)
        self.y = y(self.y, l, self.theta)
        r=rot(self.outl, self.outr, 1)
        self.theta = theta(self.theta, r)


        print(self.measures.time)
        print(f"X calculé : {self.x}, X mesuré : {round(self.measures.x,2)}, X relative : {self.measures.x - self.startValues['x']}")
        print(f"Y calculé : {self.y}, Y mesuré : {self.measures.y-self.startValues['y']}")
        # These statements make the robot rotate when there is a wall in front
        if    self.measures.irSensor[center_id]  > 1.1\
            and self.measures.irSensor[right_id] > self.measures.irSensor[left_id]\
            and self.measures.irSensor[right_id] > 1.15:
            drive_left = -0.15
            drive_right = 0.15
        elif    self.measures.irSensor[center_id]  > 1.1\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]\
            and self.measures.irSensor[left_id] > 1.15:
            drive_left = 0.15
            drive_right = -0.15

        # Security statements, to avoid collision with a lateral wall
        elif self.measures.irSensor[left_id]> 15.0\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]:
            drive_left = 0.15
            drive_right=-0.05
        elif self.measures.irSensor[right_id]> 15.0\
            and self.measures.irSensor[right_id]   > self.measures.irSensor[left_id]:
            drive_left = -0.05
            drive_right= 0.15
        
        # Security statements, to avoid collision with a front wall
        elif self.measures.irSensor[center_id]> 2.0\
            and self.measures.irSensor[left_id]   > self.measures.irSensor[right_id]:
            drive_left = 0.15
            drive_right = -0.15
        elif self.measures.irSensor[center_id]> 2.0\
            and self.measures.irSensor[right_id]   > self.measures.irSensor[left_id]:
            drive_left = -0.15
            drive_right = 0.15
            
        else:
            drive_left = 0.15
            drive_right = 0.15
        
        self.driveMotors(drive_left,drive_right)

        self.drove_left = drive_left
        self.drove_right = drive_right
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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
