import cv2
import time
import numpy as np
import pypot.dynamixel
import time
import numpy as np
import xml.etree.cElementTree as ET
import Adafruit_ADXL345
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
tilt = GPIO.PWM(7,50)
tilt.start(6)
time.sleep(1)
tilt.stop()

#Servo Class to control head microservos
'''
class servo(object):
    def __init__(self,id):
        self.id = id
        GPIO.setup(self.id,GPIO.OUT)
        self.pwm = GPIO.PWM(self.id,50)
        self.pwm.start(7.5)
        self.pwm.stop()

    def setangle(degrees):
        degrees = np.linspace(2.5,12.5,180)[degrees]
        self.pwm.start(degrees)
        time.sleep(0.01)
        self.pwm.stop()


pan = servo(7)
tilt = servo(11)
'''
#Open XML File in which angles are stored
tree = ET.ElementTree(file='data2.xml')
tree2 = ET.ElementTree(file='DRIBLE.xml')

#Dynamixel Code
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise IOError('no port found!')

print('ports found', ports)
print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])
ids = dxl_io.scan(range(25))
print(ids)

if len(ids) < 18:
	print("Failed")
	exit()

raw_input("Proceed?")

#Image Processing Setup
cx,cy = 0,0
cap = cv2.VideoCapture(0)
x,y,z = cap.get(3),cap.get(4),cap.get(15)
print x,y,z
cap.set(3,320)
cap.set(4,240)
cap.set(15,-5)
#orangeLower=(100,100,100)
#orangeUpper=(140,255,255)
ballower = np.array([60,100,100])
ballupper = np.array([74,255,255])
#botlower=np.array([100,100,100])
#botupper=np.array([130,255,255])



cx,cy = 0,0
#Function that takes an image,detects whether green ball is present.
def img(mode):
    print "Scanning"
    global cx,cy
    _,f = cap.read()
    if mode == 'optimize':
	f = f[max(cx-20,0):min(cx+20,320),max(cy-20,0):min(cy+20,240)]
    f = cv2.flip(f,1)
    #blur = cv2.medianBlur(f,5)
    try:
	hsv = cv2.cvtColor(f,cv2.COLOR_BGR2HSV)
    except:
	return 'failed'

    mask = cv2.inRange(hsv, ballower, ballupper)
    erode = cv2.erode(mask,None,iterations = 2)

    dilate = cv2.dilate(erode,None,iterations = 7)
    contours,hierarchy = cv2.findContours(dilate,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    #cv2.destroyAllWindows()
    #cap.release()
    if mode=='optimize' and len(contours)>0:
	print "BOOOM!"
	return 'success'
    if mode=='optimize' and len(contours)==0:
	return 'failed'

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x + w / 2, y + h / 2
        #if 3 < hsv.item(cy,cx,0) < 10:
            #cxo, cyo = x + w / 2, y + h / 2
        #print(cx)
        area = cv2.contourArea(c)
        if area>6000:
            return 'none' #FutureKick

        if(cx<60):
            return 'right'
        elif(60<cx<260):
            return 'walk'
        elif(cx>260):
            return 'left'
    return 'none' #NoBallFound


#actuation


#Parsexml parses the xml file tree to retrieve the reqd angles
def parsexml(text,tree):
	find = "PageRoot/Page[@name='"+text+"']/steps/step"
	motions = []
	steps = [x for x in tree.findall(find)]
	for step in steps:
		motion = step.attrib['frame'] + " " + step.attrib['pose']
		motions.append(motion)
	
	return motions

#Function to initialize bot
def initwalk():
	#pose="-81.15 80.86 -68.26 67.97 -14.65 14.36 -45.12 45.12 -1.46 1.17 -50.1 49.8 -79.69 79.39 39.55 -39.84 -1.46 1.17"
	pose="-81.15 80.86 -68.26 67.97 -14.65 14.36 -45.12 45.12 -1.46 1.17 -48.1 47.8 -84.69 84.39 44.55 -44.84 -1.46 1.17"
	pose = [float(x) for x in pose.strip().split()]
	#dxl_io.set_moving_speed(dict(zip(ids,"45")))
	for i in range(1,19,2):
		dxl_io.set_goal_position({i:pose[i-1],i+1:pose[i]})
		time.sleep(0.001)


#Sync Move generates the frames for each motion and writes each frame to motor
def syncmove(begpos,endpos,delay,speed=1):	
	begpos = begpos[1:]
	endpos = endpos[1:]

	frames = [np.linspace(x,y,delay) for x,y in zip(begpos,endpos)]
	frames = zip(*frames)
	
	i = 0
	for frame in frames:
		i=i+1
		dxl_io.set_goal_position(dict(zip(ids,frame)))
		time.sleep(0.008/speed)
		
#Functions to write an offset to leg motors
def offset(motion,inc):
	motion = [float(x) for x in motion.strip().split()]
	motion[11] += -inc
	motion[12] += inc
	motion = [str(x) for x in motion]
	motion = ' '.join(motion)
        return motion

def offset2(motion):
	motion = [float(x) for x in motion.strip().split()]
	motion[1] += 6
	motion[2] += -6
	motion[11] += -7
	motion[12] += 7
	motion[13] += -7
	motion[14] += 7
	motion = [str(x) for x in motion]
	motion = ' '.join(motion)
	return motion

#Bends the torso forward/backward based on IMU Value
def stabilize(motion):
	motion = [float(x) for x in motion.strip().split()]
	z = checkorient()
	motion[11] += -1*(z/25.0)
	motion[12] += (z/25.0)
	motion = [str(x) for x in motion]
	motion = ' '.join(motion)
	return motion

#Passes each motion from a set of motions to syncmove^ function
def syncset(motions,speed):
	global stb
	frame = 0
	prev = [0, -81.15, 80.86, -68.26, 67.97, -14.65, 14.36, -45.12, 45.12, -1.46, 1.17, -50.1, 49.8, -79.69, 79.39, 39.55, -39.84, -1.46, 1.17]
	for motion in motions:
		z = checkorient()
		if abs(z)< 100:
			motion = stabilize(motion)
		motion = [float(x) for x in motion.strip().split()]
		delay = motion[0] - frame
		frame = motion[0]
		syncmove(prev,motion,delay,speed)
		prev = motion
		

#Functions for various motions defined below
def deredy():
	motions = parsexml("43 D_Ready      ",tree2)
	syncset(motions,1)

def deright():
	motions = parsexml("44 D_Right      ",tree2)
	syncset(motions,0.3)

def balance():
	motions = parsexml("2 Balance",tree2)
	syncset(motions,1)

def rturn():
	motions = parsexml("27 RT",tree2)
	motions =[offset(m,4) for m in motions]
	syncset(motions,1)
def lturn():
	motions = parsexml("28 LT",tree2)
	motions =[offset(m,4) for m in motions]
	motions =[stabilize(m) for m in motions]
	syncset(motions,1)



	
def w1():
	motions = parsexml("32 F_S_L",tree)
	motions =[offset(m,6) for m in motions]
	motions =[stabilize(m) for m in motions]
	syncset(motions,2.1)
def w2():
	motions = parsexml("33 ",tree)
	motions =[offset(m,6) for m in motions]
	motions =[stabilize(m) for m in motions]
	syncset(motions,2.5)
	
def w3():
	motions = parsexml("38 F_M_R",tree)
	motions =[offset(m,4) for m in motions]
	#motions =[stabilize(m) for m in motions]
	syncset(motions,3)

def w4():
	motions = parsexml("39 ",tree)
	motions =[offset(m,4) for m in motions]
	#motions =[stabilize(m) for m in motions]
	syncset(motions,3)

def w5():
	motions = parsexml("36 F_M_L",tree)
	motions =[offset(m,4) for m in motions]
	#motions =[stabilize(m) for m in motions]
	syncset(motions,3)

def w6():
	motions = parsexml("37 ",tree)
	motions =[offset(m,4) for m in motions]
	#motions =[stabilize(m) for m in motions]
	syncset(motions,3)
	
def bgetup():
	motions = parsexml("28 B getup  ",tree)
	syncset(motions,1.5)
	


def fgetup():
	motions = parsexml("27 F getup",tree)
        motions = [offset2(m) for m in motions]
	syncset(motions,2.4)

def rkick():
	motions = parsexml("19 R kick",tree)
	syncset(motions,1.7)



def lkick():
	motions = parsexml("18 L kick",tree)
	syncset(motions,1.7)

def lstep():
        motions = parsexml("40 F_E_L",tree)
	syncset(motions,2.7)
        motions = parsexml("41 ",tree)
	syncset(motions,2.7)

def rstep():
        motions = parsexml("42 F_E_R",tree)
	syncset(motions,2.7)
        motions = parsexml("43 ",tree)
	syncset(motions,2.7)


def walk(iter,prev):
	if prev!='walk':
		w1()
		w2()
	while iter>0:
		w3()
		w4()
		w5()
		w6()
		iter = iter -1

	
#Function for obtaining IMU value using accelometer library
def checkorient():
	accel = Adafruit_ADXL345.ADXL345()
	#accel.set_data_rate(Adafruit_ADXL345.ADXL345_DATARATE_800HZ)
	x, y, z = accel.read()
    	return x


#If the bot falls,calls appropriate getup function
def checkfall():
	x  = checkorient()
	if abs(x)<210:
		pass
		#print "Standing"
	elif x < 0:
		print "Front Getup"
		if checkorient() < 0:
			pass
		#time.sleep(0.5)
		#fgetup()
		#time.sleep(0.5)
	else:
		time.sleep(0.5)
		if checkorient() > 210:
			bgetup()
		time.sleep(0.65)
		initwalk()
		#print "Back Getup"


#Backstep Function - Currently not being used
def b():
       # motions = parsexml("130 BLT_S_R",tree)
	#syncset(motions,1.5)
        #motions = parsexml("131 ",tree)
	#syncset(motions,1.5)
	#time.sleep(0.2)
        #motions = parsexml("140 BRT_S_L",tree)
	#syncset(motions,1)
        motions = parsexml("141 ",tree)
	motions =[stabilize(m) for m in motions] 
	syncset(motions,2)
	print "BOOM!Back!"
	 
#Balance Function - Currently not being used
def bal():
    z = checkorient()
    if z>158:
	b()
	time.sleep(0.2)
	w2()
	time.sleep(0.2)
	return True
    return False	
    
def search():
    pass	

#main
checkfall()
initwalk()
time.sleep(1.5)
counter = 3

#img function returns a value - 1/2/3/4 based on which bot
#walks forward or turns left/right or does nothing
prev = 'none' 
while True:	
    checkfall()
    action = img('optimize')
    if action=='success':
	action = prev
    else:
	action = img('normal')
    
    #action = 'walk'
    if action == 'right':
	if prev == 'walk':
	    initwalk()
	    time.sleep(.65)
	    counter = 3
        rturn()
	prev = 'right'

    elif action == 'left':
        if prev == 'walk':
	    initwalk()
	    time.sleep(.65)
	    counter = 3
	lturn()
	prev = 'left'

    elif action == 'walk':
        if prev=='left':
	    time.sleep(0.65)
	    initwalk()
	    time.sleep(1)
	if prev=='right':
	    time.sleep(0.65)
	    initwalk()
	    time.sleep(1)
	counter -= 1
    	if counter==0:
	    w2()
	    time.sleep(1.5)
	    counter = 3 
	walk(1,prev)
	prev = 'walk'
    else:
	pass

#Things yet to be added:
#1)Pan Tilt camera motions using Servo Class
#2)Goalpost detection Code
#3)Positioning and Ball Kicking
