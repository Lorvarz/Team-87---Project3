from setup import *
from random import uniform



totalTime = 10
iteration = 0
elapsed = 0
xVals = np.array([])
yVals = np.array([])
zVals = np.array([])
rotationVals = np.array([])

logFile = open("log.txt", "w")

while elapsed < totalTime:
    # if iteration == 100:
    #     drive.setAllPowers(30)
    #     drive.update()
    
    # if iteration == 160:
    #     drive.setAllPowers(0)
    #     drive.update()

    Sensor.Inertial.update()
    xVals = np.append(xVals, Sensor.Inertial.gyro[0])
    yVals = np.append(yVals, Sensor.Inertial.gyro[1])
    zVals = np.append(zVals, Sensor.Inertial.gyro[2])

    rotationVals = np.append(rotationVals, Sensor.Inertial.rotation)

    # print(Sensor.Inertial.gyro, "\n")

    
    elapsed += period
    iteration += 1
    time.sleep(period)


# for i in range(len(xVals)-1):
#     logFile.write(str(xVals[i]) + ", ")
# logFile.write(str(xVals[-1]) + "\n")

# for i in range(len(yVals)-1):
#     logFile.write(str(yVals[i]) + ", ")
# logFile.write(str(yVals[-1]) + "\n")

# for i in range(len(zVals)-1):
#     logFile.write(str(zVals[i]) + ", ")
# logFile.write(str(zVals[-1]) + "\n")

for i in range(len(rotationVals)-1):
    logFile.write(str(rotationVals[i]) + ", ")
logFile.write(str(rotationVals[-1]) + "\n")


logFile.close()

raise KeyboardInterrupt
