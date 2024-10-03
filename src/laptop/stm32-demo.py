import serial
import struct
import numpy as np
import cv2
import time
import sys

frontal_haar_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

OV7725_IMG_COLS = 240
OV7725_IMG_ROWS = 320
OV7725_IMG_CHANNELS = 3
PIXELS_PER_SEG = 4800   # Assume each pixel takes 2 bauds (16 bits)
BYTES_PER_SEG = 2

PACKET_START = 0xAABB  # Packet start header

seg_cols = PIXELS_PER_SEG // OV7725_IMG_ROWS  
seg_rows = OV7725_IMG_ROWS
seg_count = OV7725_IMG_COLS * OV7725_IMG_ROWS // PIXELS_PER_SEG

img = np.zeros((OV7725_IMG_COLS, OV7725_IMG_ROWS, OV7725_IMG_CHANNELS), dtype=np.uint8)
ser = serial.Serial("COM7", 115200, timeout=1)



def locate_packet_start():
    while True:
        b1 = ser.read()
        if ord(b1) == 0xAA:
            b2 = ser.read()
            if ord(b2) == 0xBB:
                return

while True:
    seg_id = 0

    while seg_id < seg_count:

        locate_packet_start()

        rx_id = ser.read()
        rx_id = np.frombuffer(rx_id, dtype=np.uint8)

        if rx_id != seg_id:
            seg_id = 0

        while ser.inWaiting() < PIXELS_PER_SEG * BYTES_PER_SEG:
            time.sleep(.001)

        rx_buffer = ser.read(PIXELS_PER_SEG * BYTES_PER_SEG)
        rx_buffer = np.frombuffer(rx_buffer, dtype=np.uint16)
        rgb565 = rx_buffer.reshape(seg_cols, seg_rows)
        
        # Attach current segment to img
        r_ch = (255/31 * (rgb565 >> 11)).astype(np.uint8)
        g_ch = (255/63 * (rgb565 << 5 >> 10)).astype(np.uint8)
        b_ch = (255/31 * (rgb565 << 11 >> 11)).astype(np.uint8)
        img[seg_id * seg_cols: (seg_id + 1) * seg_cols, :, :] = np.stack((b_ch, g_ch, r_ch), axis=2)

        sys.stdout.write("segment received, seg_id = " + str(rx_id) + '\n')
        seg_id = seg_id + 1

    # Perform face detection if entire image is ready
    img = cv2.flip(img, 0)
    img = cv2.flip(img, 1)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    frontal_faces = frontal_haar_cascade.detectMultiScale(img_gray, scaleFactor=1.1, minNeighbors=3, minSize=(15, 15))

    if len(frontal_faces) > 0:
        largest_area = 0
        largest_face = None

        for (x, y, width, height) in frontal_faces:
            if width * height > largest_area:
                largest_area = width * height
                largest_face = (x, y, width, height)

        tx_buffer = struct.pack('iiii', *largest_face)
        ser.write(tx_buffer)
        print(frontal_faces)
        sys.stdout.write(str(tx_buffer))
        sys.stdout.write("Face is detected!\n")
        