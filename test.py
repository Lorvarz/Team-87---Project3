# import matplotlib.pyplot as  plt
# import numpy as np

# targetVel = 21.9
# deceleration = 2.47

# f = lambda position : (-targetVel / 40 ** deceleration) * (position - 250) ** deceleration + targetVel

# x = np.arange(250, 290, 0.1)
# y = [f(flix) for flix in x]

# plt.plot(x, y)

# plt.show()

from setup import *

drive.setAllPowers(70)

input("stop")

drive.setAllPowers(0)
BP.reset_all()

