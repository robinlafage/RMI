
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14
SPEED = 0.15
OFFSET = 0.04

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, beacons_positions, start_positions, visiting_beacon):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.beacons_positions  = beacons_positions
        self.start_positions = start_positions
        self.visiting_beacon = visiting_beacon

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
        if self.measures.ground <= 0 : 
            for index, beacon in enumerate(self.visiting_beacon) :
                if beacon == True :
                    self.visiting_beacon[index] = False
        self.beaconsManagement()
        self.motorsCommand()
        print(str(self.beacons_positions))
        
        
    def motorsCommand(self) : 
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        if self.measures.irSensor[center_id] > 2 :
            if self.measures.irSensor[left_id] > self.measures.irSensor[right_id] :
                self.driveMotors(0.15,-0.15)
            elif self.measures.irSensor[right_id] > self.measures.irSensor[left_id] :
                self.driveMotors(-0.15,0.15)
        elif self.measures.irSensor[left_id] > 8 :
            self.driveMotors(0.15, 0.0)
        elif self.measures.irSensor[right_id] > 8 :
            self.driveMotors(0.0,0.15)
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
            # print("Adjusting direction - turning right")
            self.driveMotors(SPEED, SPEED-OFFSET)

        elif (dir >= -45 and dir < 0) or (dir >= 45 and dir < 90) or (dir >= 135 and dir < 180 and dir > 0) or (dir >= -135 and dir < -90):
            #Turn slowly left
            # print("Adjusting direction - turning left")
            self.driveMotors(SPEED-OFFSET, SPEED)

    def previousPositionsStack(self):
        #TODO Implement a stack to memorize the previous positions of the robot
        pass


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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host,{}, [], [])
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
