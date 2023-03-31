from setup import *

scale = float(input("scale: "))

while True:

    point = input("point: ")
    while point != "":
        #separate x and y position from ordered pair (x,y)
        x = int(point.split(",")[0])
        y = int(point.split(",")[1])

        trueX = x * scale
        trueY = y * scale

        print("Running Y")

        if (trueY > Position.y):
            turnToHeading(0)
            while(Position.y < (trueY - 0.1)):
                driveStraight(60)
                sleep(period)
                Position.update()
                print(Position)
        elif (trueY < Position.y):
            turnToHeading(180)
            while(Position.y > (trueY + 0.1)):
                driveStraight(60)
                sleep(period)
                Position.update()
                print(Position)

        print("Running X")

        if (trueX > Position.x):
            turnToHeading(60)
            while(Position.x < (trueX - 0.1) ):
                driveStraight(60)
                sleep(period)
                Position.update()
                print(Position)
        elif (trueX < Position.x):
            turnToHeading(-60)
            while(Position.x > (trueX + 0.1)):
                driveStraight(60)
                sleep(period)
                Position.update()
                print(Position)

        print("Running X")

        