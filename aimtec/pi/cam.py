import picamera
from time import sleep


with picamera.PiCamera() as camera:
        camera.preview_fullscreen=False
        camera.preview_window=(200, 50, 640, 480)

        camera.resolution=(640,480)
        camera.start_preview()
        camera.sharpness = 10
        camera.contrast = 30
        camera.vflip=False
        camera.hflip=False
        camera.exposure_mode = 'auto'

        sleep(180) # sleep for 3 minutes
        camera.stop_preview()
        camera.close()
        # camera test end