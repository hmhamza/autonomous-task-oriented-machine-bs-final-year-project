import datetime
import time
import RPi.GPIO as GPIO
import numpy as np
import threading
import motor
import web
try:
	import Queue as Q	#ver. < 3.0
except ImportError:
	import queue as Q

from collections import defaultdict

isEnd=0
Left=100
isLocalization=0
isRepeatObstacle=0

locationIndex=2	#Global Starting Point (Liberty Gate#2)

'''=========================================================='''
'''=========================================================='''
'''==================Task Manipulation Code=================='''
'''=========================================================='''
'''=========================================================='''

taskQ=Q.PriorityQueue()

def addTask(priority,source,destination):
	global taskQ
	taskQ.put((int(priority),int(source),int(destination)))

def getTask():
	global taskQ
	if (taskQ.empty()):
		return 0
	return taskQ.get()

'''=========================================================='''
'''=========================================================='''
'''===============Inter Process Communication================'''
'''=========================================================='''
'''=========================================================='''

from multiprocessing.connection import Listener

# client
def child(conn):
	msg = conn.recv()
	addTask(msg[0],msg[1],msg[2])
	conn.send("Done")

# server
def mother(address):
    serv = Listener(address)
    while True:
        client = serv.accept()
        child(client)

'''=========================================================='''
'''=========================================================='''
'''=====================Data Getter Code====================='''
'''=========================================================='''
'''=========================================================='''

class serverThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
	mother(('', 5000))

'''=========================================================='''
'''=========================================================='''
'''====================Localization Code====================='''
'''=========================================================='''
'''=========================================================='''

#Loading All Map files
BitMap=np.loadtxt("bitMap.txt")
NNMap=np.loadtxt("neuralNetworkMap.txt")
ActualMap=np.loadtxt("ATOM_Map.txt")

class Location(object):
    initialIndex=None
    Name=None

def SetValues(LocationArray):
	LocationArray[0].initialIndex=0
	LocationArray[0].finalIndex=3
	LocationArray[0].Name="Liberty Gate#1"
	
	LocationArray[1].initialIndex=4
	LocationArray[1].finalIndex=5
	LocationArray[1].Name="Liberty Corridor Gate"
	
	LocationArray[2].initialIndex=6
	LocationArray[2].finalIndex=8
	LocationArray[2].Name="Liberty Gate#2"
	
	LocationArray[3].initialIndex=9
	LocationArray[3].finalIndex=11
	LocationArray[3].Name="Liberty Corridor"
	
	LocationArray[4].initialIndex=12
	LocationArray[4].finalIndex=21
	LocationArray[4].Name="Badminton Court Turn"
	
	LocationArray[5].initialIndex=22
	LocationArray[5].finalIndex=25
	LocationArray[5].Name="Video Conference Corridor"
	
	LocationArray[6].initialIndex=26
	LocationArray[6].finalIndex=28
	LocationArray[6].Name="Video Conference Room"
	
	LocationArray[7].initialIndex=29
	LocationArray[7].finalIndex=31
	LocationArray[7].Name="Dr. A.D.Raza Office"
	
	LocationArray[8].initialIndex=32
	LocationArray[8].finalIndex=33
	LocationArray[8].Name="Dr. S.M. Sajid Office"
	
	LocationArray[9].initialIndex=34
	LocationArray[9].finalIndex=36
	LocationArray[9].Name="Raheela Tariq Office"
	
	LocationArray[10].initialIndex=37
	LocationArray[10].finalIndex=40
	LocationArray[10].Name="Sciences & Humanities Corridor"
	
	LocationArray[11].initialIndex=41
	LocationArray[11].finalIndex=48
	LocationArray[11].Name="Sciences & Humanities Department"
	
	LocationArray[12].initialIndex=49
	LocationArray[12].finalIndex=56
	LocationArray[12].Name="Manager Admin & Accounts"
	
	LocationArray[13].initialIndex=57
	LocationArray[13].finalIndex=61
	LocationArray[13].Name="Reception"
	
	LocationArray[14].initialIndex=62
	LocationArray[14].finalIndex=66
	LocationArray[14].Name="Behind the Stairs"
	
	LocationArray[15].initialIndex=67
	LocationArray[15].finalIndex=71
	LocationArray[15].Name="Stairs"
	
	LocationArray[16].initialIndex=72
	LocationArray[16].finalIndex=76
	LocationArray[16].Name="After Stairs"
	
	LocationArray[17].initialIndex=77
	LocationArray[17].finalIndex=80
	LocationArray[17].Name="Lab 2-3 Corridor#1"
	
	LocationArray[18].initialIndex=81
	LocationArray[18].finalIndex=84
	LocationArray[18].Name="Lab 2-3 Corridor#2"
	
	LocationArray[19].initialIndex=85
	LocationArray[19].finalIndex=88
	LocationArray[19].Name="Lab 2-3 Corridor#3"
	
	LocationArray[20].initialIndex=89
	LocationArray[20].finalIndex=92
	LocationArray[20].Name="Lab 2-3 Corridor#4"
	
	LocationArray[21].initialIndex=93
	LocationArray[21].finalIndex=96
	LocationArray[21].Name="Lab 2-3 Corridor#5"
	
	LocationArray[22].initialIndex=97
	LocationArray[22].finalIndex=100
	LocationArray[22].Name="Lab 2-3 Corridor#6"
	
	LocationArray[23].initialIndex=101
	LocationArray[23].finalIndex=106
	LocationArray[23].Name="Lab 2-3 Corridor#7"
	
	LocationArray[24].initialIndex=107
	LocationArray[24].finalIndex=115
	LocationArray[24].Name="Lab 2-3"
	
	LocationArray[25].initialIndex=116
	LocationArray[25].finalIndex=124
	LocationArray[25].Name="Lab 1-2-3 Turn"
	
	LocationArray[26].initialIndex=125
	LocationArray[26].finalIndex=129
	LocationArray[26].Name="Lab 1"

def bitMapRowGenerator(Map):
	ObstacleLimit=250
	currentNumber=0;
	for j in range(0,17):
		currentNumber=(currentNumber<<1)|(Map[j]<ObstacleLimit)
	return currentNumber

def findLocationIndex(index):
	global LocationArray
	for i in range(27):
		if (LocationArray[i].initialIndex<=index and LocationArray[i].finalIndex>=index):
			return i
	return -1

def Turn():
	StepPins=[10,9,11,25]
	MapArray=np.zeros(17)
	DataArray=np.zeros(5)
	
	for pin in StepPins:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin,False)
	
	StepCounter=0
	WaitTime=0.0015
	
	StepCount2=8
	Seq2=[]
	Seq2=range(0,StepCount2)
	Seq2[0]=[1,0,0,0]
	Seq2[1]=[1,1,0,0]
	Seq2[2]=[0,1,0,0]
	Seq2[3]=[0,1,1,0]
	Seq2[4]=[0,0,1,0]
	Seq2[5]=[0,0,1,1]
	Seq2[6]=[0,0,0,1]
	Seq2[7]=[1,0,0,1]
	
	Seq=Seq2
	StepCount=StepCount2
	count=0
	i=0
	j=0
	
	while i<16:
		for ReadingCount in range(0,5):
			DataArray[ReadingCount]=reading(24,17)
		MapArray[i]=np.median(DataArray)
		time.sleep(1)
		while j<128:
			for pin in range(0,4):
				xpin=StepPins[pin]
				if Seq[StepCounter][pin]!=0:
					GPIO.output(xpin,True)
				else:
					GPIO.output(xpin,False)
			StepCounter+=1
	
			if (StepCounter==StepCount):
				StepCounter=0
			if (StepCounter<0):
				StepCounter=StepCount
	
			time.sleep(WaitTime)
			j+=1
		j=0
		i+=1
	
	for ReadinCount in range(0,5):
		DataArray[ReadingCount]=reading(24,17)
	MapArray[i]=np.median(DataArray)
	StepCounter=StepCount2-1
	i=0
	while i<512*4:
		for pin in range(0,4):
			xpin=StepPins[pin]
			if Seq[StepCounter][pin]!=0:
				GPIO.output(xpin,True)
			else:
				GPIO.output(xpin,False)
		StepCounter-=1

		if (StepCounter==-1):
			StepCounter=StepCount2-1
		time.sleep(WaitTime)
		i+=1
	
	for pin in StepPins:
		GPIO.output(pin,False)
	
	return MapArray

def Localization():
	global locationIndex
	#Peforming Localization Multi-Step Process
	MapDimensions=BitMap.shape
	indices=[i for i in range(MapDimensions[0]+1)]
	indices[MapDimensions[0]]=-1
	totalReps=3
	marginForBitError=totalReps-1
	index=0 #Only for readings array
	bitReading=np.zeros(totalReps)
	readings=np.zeros((totalReps,17))

	while (marginForBitError>=0):
		readings[index]=np.copy(Turn())
		print readings[index]
		ComparedBitValues=np.zeros(MapDimensions[0])
		bitReading[index]=bitMapRowGenerator(readings[index])
		
		counter=0
		i=indices[counter]
		while (i!=-1):
			if (i<MapDimensions[0]):
				ComparedBitValues[i]=17-(bin(int(bitReading[index])^int(BitMap[i])).count('1'))
			counter=counter+1
			i=indices[counter]
		print ComparedBitValues
		
		if (marginForBitError>0):
			counter=0
			localmax=np.max(ComparedBitValues)
			for i in range(0,MapDimensions[0]):
				if (ComparedBitValues[i]+marginForBitError>=localmax):
					indices[counter]=i+1
					counter=counter+1
			indices[counter]=-1
			time.sleep(0.5)
			motor.isObstacle=0
			motor.forward(0.75)
			time.sleep(0.5)
		else:
			time.sleep(0.5)
			motor.isObstacle=0
			motor.backward(1.5)
			time.sleep(0.5)

		marginForBitError=marginForBitError-1
		index=index+1
	
	sI=LocationArray[locationIndex].initialIndex
	fI=LocationArray[locationIndex].finalIndex
	count=0
	for i in range(sI,fI+1):
		if (ComparedBitValues[i]>=14):
			count+=1
	if (count==0):
		newIndex=-1
		for x in range(1,11):
			if (sI-x>=0 and ComparedBitValues[sI-x]>=14):
				newIndex=sI-x
				break
			if (fI+x>=0 and ComparedBitValues[fI+x]>=14):
				newIndex=fI+x
				break
		if (newIndex<0):
			newIndex=np.max(ComparedBitValues)
		
		locationIndex=findLocationIndex(newIndex-2)
			

'''=========================================================='''
'''=========================================================='''
'''===================Path Calculator Code==================='''
'''=========================================================='''
'''=========================================================='''

Graph = defaultdict(dict)
allNodes=[]

class Move(object):
    def __init__(self,direction,distance,arrived):
        self.direction=direction
        self.distance=distance
        self.arrived=arrived

def populateGraph():
    fin=open("Path.txt","r")
    filetext=fin.read()
    allLines=filetext.split('\n')

    for line in allLines:
        args=line.split(',')
        allNodes.append(args[0])
        Graph[args[0]]={0:args[1],1:args[2],2:args[3],3:args[4],4:args[5]}
        if args.__len__()==7:
            Graph[args[0]][5]=args[6]

def findPath(source,destination):
    DijkstraTable = defaultdict(dict)
    nodesQueue=[]

    #Initialization for Dijkstra Algo
    for g in Graph:
        nodesQueue.append(g)
        DijkstraTable[g]={0:"NULL",1:float("inf")}

    DijkstraTable[source][0]="START";
    DijkstraTable[source][1]=0;

    ###############################################

    while nodesQueue.__len__()>0:

        #Extracting Cheapest element from the Queue
        minVal=float("inf")
        u=None
        for key in DijkstraTable:
            if key in nodesQueue:
                if DijkstraTable[key][1]<minVal:
                    u=key
                    minVal=DijkstraTable[key][1]
        nodesQueue.remove(u)
        ###########################################

        #Relaxing edges of u
        for i in range(1,5):
		
		if (Graph[u][i].__getitem__(0)=="X"):
			Graph[u][i]="X"
                if Graph[u][i]!="X":
                	v= Graph[u][i]
	                w=int(Graph[u][0])

			#print v
			#print DijkstraTable[v]
			#print DijkstraTable[v][1]
        	        if (DijkstraTable[v][1] > DijkstraTable[u][1] + w):
                		DijkstraTable[v][1]=DijkstraTable[u][1] + w
				DijkstraTable[v][0]=u
        ############################################

    #Developing the final path from the Dijkstra Algo. Table

    stack=[]
    node=destination
    nextNode=DijkstraTable[destination][0]

    while nextNode!=source:
        if Graph[nextNode][1]==node:
            stack.append(Move("FORWARD",Graph[nextNode][0],node))
        elif Graph[nextNode][2]==node:
            stack.append(Move("BACKWARD",Graph[nextNode][0],node))
        elif Graph[nextNode][3]==node:
            stack.append(Move("LEFT",int(Graph[nextNode][0])-int(Graph[nextNode][5]),node))
            stack.append(Move("FORWARD",Graph[nextNode][5],nextNode+" Junction"))
        elif Graph[nextNode][4]==node:
            stack.append(Move("RIGHT",int(Graph[nextNode][0])-int(Graph[nextNode][5]),node))
            stack.append(Move("FORWARD",Graph[nextNode][5],nextNode+" Junction"))

        node=nextNode
        nextNode=DijkstraTable[node][0]

    if Graph[nextNode][1]==node:
        stack.append(Move("FORWARD",Graph[nextNode][0],node))
    elif Graph[nextNode][2]==node:
        stack.append(Move("BACKWARD",Graph[nextNode][0],node))
    elif Graph[nextNode][3]==node:
        stack.append(Move("LEFT",int(Graph[nextNode][0])-int(Graph[nextNode][5]),node))
        stack.append(Move("FORWARD",Graph[nextNode][5],nextNode+" Junction"))
    elif Graph[nextNode][4]==node:
        stack.append(Move("RIGHT",int(Graph[nextNode][0])-int(Graph[nextNode][5]),node))
        stack.append(Move("FORWARD",Graph[nextNode][5],nextNode+" Junction"))
    stack.append(Move("START",0,source))

    ############################################################################################

    #Reversing the Path if the Destination is located backwards from the Source
    if stack[0].direction=="BACKWARD":
        tempQueue=[]
        tempQueue.insert(0,Move("TURN 180",0,stack.__getitem__(stack.__len__()-1).arrived))

        prev1=None
        prev2=source
        for i in range(0,stack.__len__()):
            temp=stack.pop()
            if temp.direction=="START":
                tempQueue.insert(0,Move("START",temp.distance,temp.arrived))
            elif prev1!=None and prev1==Graph[prev2][4]:
                tempQueue.insert(0,Move("LEFT",int(temp.distance)-int(Graph[prev1][0]),temp.arrived))
            elif prev1!=None and prev1==Graph[prev2][3]:
                tempQueue.insert(0,Move("RIGHT",int(temp.distance)-int(Graph[prev1][0]),temp.arrived))
            else:
                tempQueue.insert(0,Move("FORWARD",temp.distance,temp.arrived))

            prev1=prev2
            prev2=temp.arrived
	tempQueue.insert(0,Move("TURN 180",0,""))
        stack=tempQueue
    ###############################################################################################

    return stack

'''=========================================================='''
'''=========================================================='''
'''===============Running Car for 1 Task Code================'''
'''=========================================================='''
'''=========================================================='''

def RunTheCar(source,destination):
	global isEnd
	global isLocalization
	global LocationArray
	PATH=findPath(LocationArray[source].Name,LocationArray[destination].Name)

	isEnd=0
	
	forObstacleTurn(0)
	
	threads = []
	
	# Create new threads
	thread1 = myThread(1, "Front Sensor Thread", FrontSensorPins)
	#thread2 = myThread(2, "Left Sensor Thread", LeftSensorPins)
	
	# Start new Threads
	thread1.start()
	#thread2.start()
	
	# Add threads to thread list
	threads.append(thread1)
	#threads.append(thread2)

	for i in range(0,PATH.__len__()):
		temp=PATH.pop()

		string="   "+temp.direction
		
		if temp.direction=="START":
		    string=string+"         "
		elif temp.direction=="TURN 180":
		    string=string+"        "
		    motor.Turn180()
	            time.sleep(1)
		elif temp.direction=="FORWARD":
		    string=string+"       "
	            motor.forward(float(temp.distance)*0.75)
		elif temp.direction=="RIGHT":
		    string=string+"         "
		    time.sleep(0.3)
		    motor.RightAndForward(float(temp.distance)*0.75)
		    time.sleep(0.3)
		elif temp.direction=="LEFT":
		    string=string+"          "
		    time.sleep(0.3)
		    motor.LeftAndForward(float(temp.distance)*0.75)
		    time.sleep(0.3)
		
		string=string+str(temp.distance)+"              "+temp.arrived
		
	forObstacleTurn(1)
	
	# Telling threads to kill themselves
	isEnd=1
	
	# Wait for all threads to complete
	for t in threads:
    		t.join()

'''=========================================================='''
'''=========================================================='''
'''=======Threads For Obstacle and Direction Correction======'''
'''=========================================================='''
'''=========================================================='''

class myThread (threading.Thread):
    def __init__(self, threadID, name, pinsArray):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.pinsArray = pinsArray
    def run(self):
	global isEnd
	global Left
	global isRepeatObstacle
        
	print "Starting " + self.name

	read=0
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	while (isEnd==0):
		if (motor.isStraight!=0):
			pause()
		read=reading(self.pinsArray[0],self.pinsArray[1])
		#print self.name+": "+str(read)+" isEndValue:"+str(isEnd)
		if (self.threadID==1):#Forward
			print read
			if (read<125):
				motor.isObstacle=1
				isRepeatObstacle=0
			elif (isRepeatObstacle==1):
				motor.isObstacle=0
			else:
				isRepeatObstacle=1
		elif (self.threadID==2):#Left
			print str(Left)+" "+str(read)
			if ((Left<50 and read<50) or (Left>130 and read>130)):#180 total, 90 Middle Corridor
				motor.isStraight=100-read
				motor.correctDirection()
				print "Out of Straight"
				time.sleep(1)
				Left=100
			else:
				Left=read

	#GPIO.cleanup()

'''=========================================================='''
'''=========================================================='''
'''===================Sensor Reading Code===================='''
'''=========================================================='''
'''=========================================================='''

def reading(InPin,OutPin):
	#(Echo,Trig)		
	#GPIO.setwarnings(False)
	#GPIO.setmode(GPIO.BCM)
	GPIO.setup(OutPin,GPIO.OUT)
	GPIO.setup(InPin,GPIO.IN)
	GPIO.output(OutPin, GPIO.LOW)
	time.sleep(0.3)

	GPIO.output(OutPin, True)
	time.sleep(0.00001)
	GPIO.output(OutPin, False)
	while GPIO.input(InPin) == 0:
	  signaloff = time.time()
	while GPIO.input(InPin) == 1:
	  signalon = time.time()
	timepassed = signalon - signaloff
	distance = timepassed * 17000
	return distance
	#GPIO.cleanup()

def pause():
	print "inPause"
	while (motor.isStraight!=0):
		time.sleep(1)

'''=========================================================='''
'''=========================================================='''
'''===Moving Front Sensor to Center for Obstacle Detection==='''
'''=========================================================='''
'''=========================================================='''

def forObstacleTurn(param):
	StepPins=[10,9,11,25]	
	for pin in StepPins:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin,False)

	WaitTime=0.0015
	
	StepCount2=8
	Seq2=[]
	Seq2=range(0,StepCount2)
	Seq2[0]=[1,0,0,0]
	Seq2[1]=[1,1,0,0]
	Seq2[2]=[0,1,0,0]
	Seq2[3]=[0,1,1,0]
	Seq2[4]=[0,0,1,0]
	Seq2[5]=[0,0,1,1]
	Seq2[6]=[0,0,0,1]
	Seq2[7]=[1,0,0,1]
	
	Seq=Seq2

	if (param==0):#ClockWise
		StepCounter=0
		i=0
		while i<256*4:
			for pin in range(0,4):
				xpin=StepPins[pin]
				if Seq[StepCounter][pin]!=0:
					GPIO.output(xpin,True)
				else:
					GPIO.output(xpin,False)
			StepCounter+=1

			if (StepCounter==StepCount2):
				StepCounter=0
			if (StepCounter<0):
				StepCounter=StepCount2
			time.sleep(WaitTime)
			i+=1
	elif (param==1):#AntiClockwise
		StepCounter=StepCount2-1
		i=0
		while i<256*4:
			for pin in range(0,4):
				xpin=StepPins[pin]
				if Seq[StepCounter][pin]!=0:
					GPIO.output(xpin,True)
				else:
					GPIO.output(xpin,False)
			StepCounter-=1

			if (StepCounter==-1):
				StepCounter=StepCount2-1
			time.sleep(WaitTime)
			i+=1
	

	for pin in StepPins:
		GPIO.output(pin,False)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

FrontSensorPins=[24,17]
#RightSensorPins=[19,13]
LeftSensorPins=[6,5]

'''=========================================================='''
'''=========================================================='''
'''====================Main Function Code===================='''
'''=========================================================='''
'''=========================================================='''

LocationArray=[Location() for i in range(27)]
SetValues(LocationArray)

def main():
	sThread = serverThread()
	sThread.start()

	global locationIndex

	populateGraph()
	
	try:
		while (True):
			taskTuple=getTask()
			while(taskTuple==0):
				taskTuple=getTask()
			print taskTuple
			
			while(taskTuple[1]!=locationIndex):
				RunTheCar(locationIndex,taskTuple[1])
				locationIndex=taskTuple[1]
				Localization()
			while(taskTuple[2]!=locationIndex):
				RunTheCar(locationIndex,taskTuple[2])
				locationIndex=taskTuple[2]
				Localization()
	
	except KeyboardInterrupt:
		GPIO.cleanup()

main()