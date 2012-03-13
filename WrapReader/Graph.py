import pygame as pg
import sys
import time
import os
from pygame.locals import *

# Commandline flags
TXT = ("-txt" in sys.argv)
PNG = ("-png" in sys.argv)

# Pygame init
dataLen = 12
(width, height) = (1000, 700)
sep = (width-100)/(dataLen-1)

pg.init()
window = pg.display.set_mode((width,height), RESIZABLE)
pg.display.set_caption("Graph")
canvas = pg.PixelArray(window)
white, gray, black = Color(255,255,255), Color(70,70,70), Color(0,0,0)

# Funcs
def screenshot():
	os.system("scrot --focused graphs/"+str(int(time.time()))+".png")
	#pg.image.save(pg.display.get_surface(), "graphs/"+str(int(time.time()))+".png")
	
def exit():
	if PNG: screenshot()
	sys.exit()

maxdata, data, mindata = [-5000]*dataLen, [0]*dataLen, [+5000]*dataLen
skip = 5
skipcnt = 0
cnt = 0
while True:
    # Handle events
    for event in pg.event.get():
        if event.type == KEYDOWN and event.key == K_ESCAPE: exit()
        if event.type == QUIT: exit()
        if event.type == VIDEORESIZE: 
            width, height = event.size
            window = pg.display.set_mode((width,height), RESIZABLE)
            sep = (width-100)/(12-1)
            pg.draw.rect(window, black, (0,0,width,height))

    # Read data
    read = sys.stdin.readline()[:-1]
    if(read==""): exit()
    if TXT: print read

    # Skip frames
    skipcnt += 1
    if not skipcnt%skip == 0: continue
    
    # Parse data
    exdata = list(data)
    data = [int(s) for s in read.split(" ")[1:]]

    # Clear line
    clear = 10
    pg.draw.line(window, black, (0,cnt-1+clear/2),(width,cnt-1+clear/2), clear)
    
    # Data lines
    for i in range(len(data)):
        maxdata[i], mindata[i] = max(data[i], maxdata[i]), min(data[i], mindata[i])

        # Normalize data somewhat
        multi = 1
        if i in [0,1,2]: multi = 4
        if i in [6,7,8]: multi = 2
        
        xorigin = 50+i*sep
        pg.draw.aaline(window, white, 
            (xorigin + multi*(width/1000.0)*(exdata[i]/50.0), cnt-1),
            (xorigin + multi*(width/1000.0)*(data[i]/50.0), cnt))
        pg.draw.line(window, gray, (xorigin,0),(xorigin,height), 1)

    pg.display.flip()
    
    cnt += 1
    if cnt>=height:
        cnt = 0

        # Save screenshot
        if PNG: screenshot()

