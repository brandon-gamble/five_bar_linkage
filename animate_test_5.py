'''


'''

from FBL_libraries import FiveBarLinkage_lib_R07 as FBL_lib
import numpy as np
from matplotlib import pyplot as plt

sys = FBL_lib.FBL([8,8,8,8,8])




xy = np.zeros([2,2])
xy[0,:] = [4, 12.304]   # normal pentagon
xy[1,:] = [4, 14.93]    # 90 degr

# xy = sys.xy_CSVToArray('two_arcs.csv')
xy = sys.xy_CSVToArray('controller.csv')


'''
xy = np.zeros([1,2])
xy[0,0] = 8
xy[0,1] = 8
'''

theta = sys.getMotorAngles(xy,'++')
theta_deg = theta*180/np.pi
abso, rela, shift = sys.motorSteps(theta, 360)

'''
print('XY points')
print(xy)
print()

print('Absolute Angles')
print(theta_deg)
print()
'''

# plot path
plt.plot(xy[:,0],xy[:,1])
plt.xlabel('$x$')
plt.ylabel('$y$')
plt.title('')
plt.show()

# plot theta signals against each other
plt.plot(theta[:,0],theta[:,1])
plt.xlabel(r'$\theta_1$')
plt.ylabel(r'$\theta_2$')
plt.show()


sys.animatePath(theta, xy,
                frameDelay=5,
                width = 2,
                save=False,
                ghost=False,
                draw=True)
