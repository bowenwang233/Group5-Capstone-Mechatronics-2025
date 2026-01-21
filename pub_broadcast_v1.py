import sys
import socket
import traceback
import cv2
from imutils.video import VideoStream
import imagezmq
import simplejpeg


PORT = 5555

JPEG_QUALITY = 70

def main():
        publisher = imagezmq.ImageSender("tcp://*:{}".format(PORT), REQ_REP=False)

        camera = VideoStream(src=1).start()

        print("Input stream opened")

        rpi_name = socket.gethostname()

        try:
                counter = 0
                while True:
                        frame = camera.read()
                        jpg_buffer = simplejpeg.encode_jpeg(frame, quality = JPEG_QUALITY, colorspace = 'BGR')

                        reply_from_pc = publisher.send_jpg(rpi_name, jpg_buffer)

        except (KeyboardInterrupt, SystemExit):
                pass
        except Exception as ex:
                print('Python error with no Exception handler:')
                print('Traceback error:', ex)
                traceback.print_exc()
        finally:
                camera.stop()
                sys.exit()


if __name__ == "__main__":
        main()
