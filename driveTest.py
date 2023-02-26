from setup import *

power = 50
while True:
    drive.setAllPowers(power)
    drive.reduceLeft(2 *  power)
    drive.update()
    sleep(period)