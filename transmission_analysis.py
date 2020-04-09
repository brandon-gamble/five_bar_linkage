from FBL_libraries import FiveBarLinkage_lib_R07 as FBL_lib
import numpy as np
from matplotlib import pyplot as plt


def buildArray(X,Y):
    xmin = X[0]
    xmax = X[1]
    numX = X[2]
    xs = np.linspace(xmin,xmax, numX)

    ymin = Y[0]
    ymax = Y[1]
    numY = Y[2]
    ys = np.linspace(ymin,ymax,numY)

    array = np.array([[xmin,ymin]])
    hold = np.zeros([len(xs),2])

    for y in ys:
        hold[:,0] = xs
        hold[:,1] = y
        array = np.append(array,hold,axis=0)

    return array


# a1 b1 // a2 b2 // w
X = 1.3
systems = np.array([
    [1, 1, 1, 1, 1],
    [1, X, 1, X, 1],
    [X, 1, 1, 1, 1],
    [1, X, 1, 1, 1],
    [1, 1, X, 1, 1],
    [1, 1, 1, X, 1],
    [1, 1, 1, 1, X],
])



numSys = np.shape(systems)[0]

test_points = buildArray([-.5,1.5,35],[.25,1.75,30])

print('Transmission Analysis')

for config in systems:
    sys = FBL_lib.FBL(config)
    mu = sys.getTransAngles(test_points)
    
    sys.transmission_plot(mu, test_points,1,75)





