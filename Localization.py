import datetime
import time
import RPi.GPIO as GPIO
import numpy as np
import motor

class Location(object):
    initialIndex=None
    Name=None

def bitMapRowGenerator(Map):
    ObstacleLimit=250
    currentNumber=0;
    for j in range(0,17):
        currentNumber=(currentNumber<<1)|(Map[j]<ObstacleLimit)
    return currentNumber

def SetValues(LocationArray):
	LocationArray[0].initialIndex=0
	LocationArray[1].initialIndex=4
	LocationArray[2].initialIndex=10
	LocationArray[3].initialIndex=13
	LocationArray[4].initialIndex=20
	LocationArray[0].Name="Liberty's 1st Gate"
	LocationArray[1].Name="Corridor Gate"
	LocationArray[2].Name="Liberty's 2nd Gate"
	LocationArray[3].Name="Corridor Before First Turn"
	LocationArray[4].Name="1st Turn - Right"

def reading():		
	# GPIO.setwarnings(False)
	# GPIO.setmode(GPIO.BCM)
	# point the software to the GPIO pins the sensor is using
	# change these values to the pins you are using
	# GPIO output = the pin that's connected to "Trig" on the sensor
	# GPIO input = the pin that's connected to "Echo" on the sensor
	GPIO.setup(17,GPIO.OUT)
	GPIO.setup(24,GPIO.IN)
	GPIO.output(17, GPIO.LOW)
	# found that the sensor can crash if there isn't a delay here
	# no idea why. If you have odd crashing issues, increase delay
	time.sleep(0.1)
	
	# sensor manual says a pulse ength of 10Us will trigger the 
	# sensor to transmit 8 cycles of ultrasonic burst at 40kHz and 
	# wait for the reflected ultrasonic burst to be received
	
	# to get a pulse length of 10Us we need to start the pulse, then
	# wait for 10 microseconds, then stop the pulse. This will 
	# result in the pulse length being 10Us.
	
	# start the pulse on the GPIO pin 
	# change this value to the pin you are using
	# GPIO output = the pin that's connected to "Trig" on the sensor
	GPIO.output(17, True)
	# wait 10 micro seconds (this is 0.00001 seconds) so the pulse
	# length is 10Us as the sensor expects
	time.sleep(0.00001)
	
	# stop the pulse after the time above has passed
	# change this value to the pin you are using
	# GPIO output = the pin that's connected to "Trig" on the sensor
	GPIO.output(17, False)

	# listen to the input pin. 0 means nothing is happening. Once a
	# signal is received the value will be 1 so the while loop
	# stops and has the last recorded time the signal was 0
	# change this value to the pin you are using
	# GPIO input = the pin that's connected to "Echo" on the sensor
	while GPIO.input(24) == 0:
	  signaloff = time.time()
	# listen to the input pin. Once a signal is received, record the
	# time the signal came through
	# change this value to the pin you are using
	# GPIO input = the pin that's connected to "Echo" on the sensor
	while GPIO.input(24) == 1:
	  signalon = time.time()
	# work out the difference in the two recorded times above to 
	# calculate the distance of an object in front of the sensor
	timepassed = signalon - signaloff
	
	# we now have our distance but it's not in a useful unit of
	# measurement. So now we convert this distance into centimetres
	distance = timepassed * 17000
	
	# return the distance of an object in front of the sensor in cm
	return distance
	
	# we're no longer using the GPIO, so tell software we're done
	# GPIO.cleanup()

def Turn():
	#GPIO.setwarnings(False)
	#GPIO.setmode(GPIO.BCM)

	StepPins=[10,9,11,25]
	MapArray=np.zeros(17)
	DataArray=np.zeros(5)
	
	for pin in StepPins:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin,False)
	
	#time.sleep(5)
	
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
			DataArray[ReadingCount]=reading()
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
		DataArray[ReadingCount]=reading()
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
	
	#GPIO.cleanup()
	return MapArray
	
def ExactSearch(Map,LocalArray,Diff,Result):	
	MapDimensions=Map.shape
	for i in range(0,MapDimensions[0]):
		Check=True
		for j in range(0,MapDimensions[1]):
			if (not(LocalArray[j]>=Map[i][j]-Diff and LocalArray[j]<=Map[i][j]+Diff)):
				Check=False
				break
		if (Check):
			Result[i]+=(50/Diff)

def MiddleSearch(Map,LocalArray,Diff,Result):
	MapDimensions=Map.shape
	for i in range(0,MapDimensions[0]):
		for j in range(0,MapDimensions[1]):
			if (LocalArray[j]>=Map[i][j]-Diff and LocalArray[j]<=Map[i][j]+Diff):
				Result[i]+=(50/Diff)
	
def NotMiddleSearch(Map,LocalArray,Diff,Result):
	MapDimensions=Map.shape
	DoubleMiddle=np.zeros(9)
	for i in range(0,MapDimensions[0]):
		for k in range(0,9):
			DoubleMiddle[k]=(Map[i][k]+Map[i][16-k])/2
		for j in range(0,9):
			if (LocalArray[j]>=DoubleMiddle[j]-Diff and LocalArray[j]<=DoubleMiddle[j]+Diff):
				Result[i]+=1
	
def MapLocationFinder(LocationArray,FoundArray,FoundValue):
	Dim=FoundArray.shape
	for i in range(0,Dim[0]):
		check=False
		if (FoundArray[i]==FoundValue):
			print i
			for j in range(0,5):
				if (i<LocationArray[j].initialIndex):
					check=True
					break
			if (check):
				print LocationArray[j-1].Name
			else:
				print LocationArray[j].Name

def main():
	#LocationArray=[Location() for i in range(5)]
	#SetValues(LocationArray)

	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)	

	Map=np.loadtxt('ATOM_Map.txt')
	
	'''
	Result0=np.zeros(MapDimensions[0])
	ExactSearch(Map,LocalArray,50,Result0)
	ExactSearch(Map,LocalArray,20,Result0)
	ExactSearch(Map,LocalArray,5,Result0)
	print Result0
	Max0=np.amax(Result0)
	MaxIndex0=np.argmax(Result0)
	print Max0
	print MaxIndex0
	#if (Max0>0):
		#MapLocationFinder(LocationArray,Result0,Max0)
	'''

	#Loading All Map files
	BitMap=np.loadtxt("bitMap.txt")
	NNMap=np.loadtxt("neuralNetworkMap.txt")
	ActualMap=np.loadtxt("ATOM_Map.txt")
	
	#Peforming Localization Multi-Step Process
	MapDimensions=BitMap.shape
	indices=[i for i in range(MapDimensions[0]+1)]
	indices[MapDimensions[0]]=-1
	totalReps=3
	marginForBitError=totalReps-1
	index=0 #Only for readings array
	bitReading=np.zeros(totalReps)
	readings=np.zeros((totalReps,17))

	'''
	Result1=np.zeros(MapDimensions[0])
	MiddleSearch(ActualMap,readings[index],50,Result1)
	MiddleSearch(ActualMap,readings[index],20,Result1)
	MiddleSearch(ActualMap,readings[index],5,Result1)
	print Result1
	Max1=np.amax(Result1)
	MaxIndex1=np.argmax(Result1)
	print Max1
	print MaxIndex1
	#MapLocationFinder(LocationArray,Result1,Max1)
	'''
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
		
		counter=0
		localmax=np.max(ComparedBitValues)
		for i in range(0,MapDimensions[0]):
			if (ComparedBitValues[i]+marginForBitError>=localmax):
				indices[counter]=i+1
				counter=counter+1
		indices[counter]=-1
		
		marginForBitError=marginForBitError-1
		index=index+1
		time.sleep(0.5)
		motor.forward(0.75)
		time.sleep(0.5)
	
	indices[0]-=1
	print indices[0]

	GPIO.cleanup()
	
	'''
	DoubleLocal=np.zeros(9)
	for i in range(0,9):
		DoubleLocal[i]=(LocalArray[i]+LocalArray[16-i])/2
	Result2=np.zeros(MapDimensions[0])
	NotMiddleSearch(Map,DoubleLocal,50,Result2)
	NotMiddleSearch(Map,DoubleLocal,20,Result2)
	NotMiddleSearch(Map,DoubleLocal,5,Result2)
	print Result2
	Max2=np.amax(Result2)
	MaxIndex2=np.argmax(Result2)
	print Max2
	print MaxIndex2
	#MapLocationFinder(LocationArray,Result2,Max2)
	'''
main()