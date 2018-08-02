from picamera import PiCamera
from time import sleep


print ("Photoshoot Starting ;)")

camera = PiCamera()
camera.resolution = (2592, 1944)
camera.framerate = 15

sleep(4)
for i in range(10):
	camera.start_preview()
	sleep(1)
	camera.capture('/home/pi/image%s.jpg' % i)
	print ('Pic Taken')
	camera.stop_preview()

print ("Photoshoot Finished")
