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
xy[:,0] -= -.80#3
xy[:,1] -= -1.75#1


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
plt.title('Desired path')
plt.show()

# plot theta signals against each other
plt.plot(theta[:,0],theta[:,1])
plt.xlabel(r'$\theta_1$')
plt.ylabel(r'$\theta_2$')
plt.title(r'$\theta_1$ vs $\theta_2$')
plt.show()


# rotate x and y
rot_angle = 135/180*np.pi
rot_mat = np.array([[np.cos(rot_angle), -np.sin(rot_angle)],
                [np.sin(rot_angle), np.cos(rot_angle)]])
xy_rot = np.matmul(rot_mat,np.transpose(xy))
xy_rot = xy_rot.transpose()

# apply transform
xy_transform = np.zeros([len(xy),2])
#xy_transform[:,0] = xy_rot[:,0]+25.1
#xy_transform[:,1] = xy_rot[:,1]+18

xy_transform[:,0] = xy_rot[:,0]+25.5
xy_transform[:,1] = xy_rot[:,1]+20

xy_transform /= 7.5

# theta1 and x
plt.plot(np.arange(len(theta)),theta[:,0], label=r'$\theta_1$')
plt.plot(np.arange(len(xy)),xy_transform[:,0], label=r'$(x_{rot}+25.5)/7.5$')
plt.legend()
plt.title(r'$\theta_1$ and Transform of X')
plt.xlabel(r't')
plt.ylabel(r'x, $\theta$')
plt.show()

# theta2 and y
plt.plot(np.arange(len(theta)),theta[:,1], label=r'$\theta_2$')
plt.plot(np.arange(len(xy)),xy_transform[:,1], label=r'$(y_{rot}+20)/7.5$')
plt.title(r'$\theta_2$ and Transform of Y')
plt.xlabel(r't')
plt.ylabel(r'y, $\theta$')
plt.legend()
plt.show()

# test approximation
theta_approx = xy_transform
xy_approx = sys.forwardKinematics(theta_approx)

plt.plot(xy[:,0],xy[:,1],alpha=.45,label='Original path')
plt.plot(xy_approx[:,0],xy_approx[:,1],label='Transform solution')
plt.title('Preliminary transform performance')
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.legend()
plt.show()

sys.animatePath(theta, xy,
                frameDelay=5,
                width = 2,
                save=False,
                ghost=False,
                draw=True)


