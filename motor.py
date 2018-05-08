import RPi.GPIO as GPIO
import datetime
import time
import threading

isObstacle=0
isStraight=0
Timer=0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#21-RightBack, 20-RightForward, 16-LeftBack, 12-LeftForward
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

threadLock = threading.Lock()

def checkStoppage(x):
	global isObstacle
	global isStraight
	while (x>0):
		time.sleep(0.1)
		if (isObstacle==1 or isStraight!=0):
			break
		x=x-0.1
	return x

def forward(x):
	global isObstacle
	global isStraight
	global Timer
	threadLock.acquire()
	GPIO.output(20,GPIO.HIGH)
	GPIO.output(12,GPIO.HIGH)
	rValue=checkStoppage(x)
	GPIO.output(20,GPIO.LOW)
	GPIO.output(12,GPIO.LOW)
	threadLock.release()
	if (rValue>0):
		while (isObstacle==1 or isStraight!=0):
			time.sleep(1)
		rValue-=Timer
		Timer=0
		forward(rValue)

def backward(x):
	global isObstacle
	global isStraight
	GPIO.output(21,GPIO.HIGH)
	GPIO.output(16,GPIO.HIGH)
	rValue=checkStoppage(x)
	GPIO.output(21,GPIO.LOW)
	GPIO.output(16,GPIO.LOW)
	if (rValue>0):
		while (isObstacle==1 or isStraight!=0):
			time.sleep(1)
		backward(rValue)

def RightAndForward(x):
	global isStraight
	#isStraight=1
	threadLock.acquire()
	
	GPIO.output(12,GPIO.HIGH)
	GPIO.output(21,GPIO.HIGH)
	time.sleep(0.6)
	GPIO.output(12,GPIO.LOW)
	GPIO.output(21,GPIO.LOW)
	time.sleep(0.1)
	
	#isStraight=0
	threadLock.release()
	forward(x)

def LeftAndForward(x):
	global isStraight
	#isStraight=1
	threadLock.acquire()

	GPIO.output(20,GPIO.HIGH)
	GPIO.output(16,GPIO.HIGH)
	time.sleep(0.6)
	GPIO.output(20,GPIO.LOW)
	GPIO.output(16,GPIO.LOW)
	time.sleep(0.1)
	
	#isStraight=0
	threadLock.release()
	forward(x)

def Turn180():
	global isStraight
	#isStraight=1
	threadLock.acquire()

	GPIO.output(12,GPIO.HIGH)
	GPIO.output(21,GPIO.HIGH)
	time.sleep(1.2)
	GPIO.output(12,GPIO.LOW)
	GPIO.output(21,GPIO.LOW)
	time.sleep(0.1)
	
	#isStraight=0
	threadLock.release()

def turnRight180(x):
	GPIO.output(12,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(12,GPIO.LOW)

def turnLeft180(x):
	GPIO.output(20,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(20,GPIO.LOW)

def turnRight360(x):
	GPIO.output(12,GPIO.HIGH)
	GPIO.output(21,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(12,GPIO.LOW)
	GPIO.output(21,GPIO.LOW)

def turnLeft360(x):
	GPIO.output(20,GPIO.HIGH)
	GPIO.output(16,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(20,GPIO.LOW)
	GPIO.output(16,GPIO.LOW)

def rightCorrection(x):
	GPIO.output(12,GPIO.HIGH)
	GPIO.output(21,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(12,GPIO.LOW)
	GPIO.output(21,GPIO.LOW)

def leftCorrection(x):
	GPIO.output(20,GPIO.HIGH)
	GPIO.output(16,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(20,GPIO.LOW)
	GPIO.output(16,GPIO.LOW)

def forwardCorrection(x):
	GPIO.output(20,GPIO.HIGH)
	GPIO.output(12,GPIO.HIGH)
	time.sleep(x)
	GPIO.output(20,GPIO.LOW)
	GPIO.output(12,GPIO.LOW)

def correctDirection():
	global isStraight
	global Timer
	threadLock.acquire()
	if (isStraight>0):
		rightCorrection(0.2)
		time.sleep(0.2)
		forwardCorrection(1)
		time.sleep(0.2)
		leftCorrection(0.2)
		Timer=1
	elif (isStraight<0):
		leftCorrection(0.2)
		time.sleep(0.2)
		forwardCorrection(1)
		time.sleep(0.2)
		rightCorrection(0.2)
		Timer=1
	isStraight=0
	#time.sleep(1)
	threadLock.release()