from PID import *

# --------------------- Sensors and PID setup --------------------------

# --* Sensors *--
# distance = Sensor(sensorType.grove, sensorType.ultraSonic, 3)
# inertial = IMUWrap()


# --* Drivetrain *--
drive = DriveTrain("B", "C")

# --* Navigation *--


# --* Gate control *--


# --------------------- Brick sensors calibration ----------------------

# try:
#     color.update()
# except brickpi3.SensorError:
#     print("Configuring...")
#     error = True
#     while error:
#         sleep(0.1)
#         try:
#             color.update()
#             error = False
#         except brickpi3.SensorError:
#             error = True
#     print("Configured!\n")


# --------------------- Line sensors & IMU calibration --------------------

biasMulti = 0.2

# input("Place MACRO on Light terrain away from magnet")
# lightValues = [color.update(), lightLeft.update(), lightRight.update()]
# # magOff = inertial.getBias()

# input("place central sensor on Line")
# color.bias = -(color.update() + (lightValues[0] - color.value) * biasMulti)

# input("place Left (front-facing) sensor on Line")
# lightLeft.bias = -(lightLeft.update() + (lightValues[1] - lightLeft.value) * biasMulti)

# input("place Right (front-facing) sensor on Line")
# lightRight.bias = -(lightRight.update() + (lightValues[2] - lightRight.value) * biasMulti)

# input("Place on magnetic beacon")
# sleep(0.6)
# inertial.update()
# inertial.magThreshold = inertial.magneticMagnitude * 0.6

# print("BIAS:", lightLeft.bias, color.bias, lightRight.bias, inertial.magThreshold)
# print(f"Threshold: {inertial.magThreshold:.2f}, {inertial.magBias}")

# ----- interrupt hook -----

def customHook(exctype, value, traceback):
    print(f"Type: {exctype}")
    if exctype == KeyboardInterrupt:
        print("Keyboard interrupt, program stopped")
        drive.setAllPowers(0)
        BP.reset_all()
        sys.exit(0)
    else:
        sys.__excepthook__(exctype, value, traceback)

sys.excepthook = customHook


# ----- cargo control functions -----

# ------


# start
delay()