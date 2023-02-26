from setup import *


try: 
    while True:
        sleep(period)

        # --- cargo control ---
        payloadManager.update()
        if payloadManager.findBase[0] and not payloadManager.findBase[1]: turnToDrop()
        if payloadManager.findDrop[0] and not payloadManager.findDrop[1]: dropCargo()
        if payloadManager.findReset: reset()

        # --- motor control ---
        finalPower = basePower

        if gyro.update()[0] > 7:
            finalPower = maxPower

        drive.setAllPowers(finalPower) 
        
        lineFollow.update()

        print(lightLeft.value <= 0, color.value <= 0, lightRight.value <= 0)

        if distance.update() <= 13: drive.setAllPowers(0)

        drive.update()

except KeyboardInterrupt:
    drive.setAllPowers(0)
    drive.update()
    BP.reset_all()
    sys.exit(0)