import picamera

camera = picamera.PiCamera()

recording =  False

while True:
	print("enter command:")
	command = input()
	if command != "lmaoooo0o0o":
		recording = not recording
	if recording:
		camera.start_preview()
	else: 
		camera.stop_preview()
