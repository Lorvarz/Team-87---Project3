from setup import *

while (True):
    target = input("What rotation? ")

    if target[0] == "-" or target[0] == "+":
        turnRelative(int(target[1:]) * (1 if target[0] == "+" else -1))
    else:
        turnToHeading(int(target))

    drive.stop()

raise KeyboardInterrupt
