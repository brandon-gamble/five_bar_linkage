
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib.patches as patches
import csv
import pandas as pd

'''
TO DO

- optimize the animation function (drawing method)
    - somehow append the trace polygon? rather than re-making it every time  and
    every time it gets much bigger.
        - use recursion for appending?
- fix animation save
- singularitites
- forward kinematics
    - +/- solution intersection detection
'''

class FBL(object):

    def __init__(self,a1,b1,a2,b2,w):
        # a is radius link from motor
        # b is floating link to end effector
        # w is ground space b/t motors
        self.a1 = a1
        self.b1 = b1
        self.a2 = a2
        self.b2 = b2
        self.w = w

        # have a function to generate singularities log in [x,y] array
        self.singularities = self.generateSingularites()

    def xy_CSVToArray(self, xy_CSV):
        '''
        Takes a csv file of xy points and converts to numpy array
        implemented with csv library
        '''
        ###########################################################
        # https://www.geeksforgeeks.org/working-csv-files-python/ #
        ###########################################################
        # csv file name
        # filename = "coordinator.csv"
        filename = xy_CSV

        # initializing the titles and rows list
        fields = []
        rows = []
        rawr =[]
        # reading csv file
        with open(filename, 'r') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)
            # extracting field names through first row
            fields = next(csvreader)

            # extracting each data row one by one
            for row in csvreader:
                rows.append(row)

        # this array is made of strings so we need to convert
        str_array = np.asarray(rows)

        # cast string array into float array
        flt_array = str_array.astype(np.float)

        return flt_array

    def xy_CSVToArrayPandas(self, xy_CSV):
        '''
        takes a csv of xy points and converts to numpy array
        implemented with pandas library
        '''
        pd_frame = pd.read_csv('coordinator_small.csv', sep=',',header=None,dtype='a')	# make a pd DataFrame
        np_str = pd.DataFrame(pd_frame).to_numpy()										# convert to np array (but strings)
        np_str_noHeader = np.delete(np_str, 0, 0)										# remove header ['x', 'y']
        np_flt = np_str_noHeader.astype(np.float)										# convert str → flt

        return np_flt

    def getMotorAngles(self, xy_array, mode):
        '''
        input: numpy array of xy points
                mode = working mode
        output: numpy array of motor angles
        '''
        # print(self.a1)
        # print(self.a2)
        # print(self.b1)
        # print(self.b2)
        # print(self.w)
        # will take a xy points and convert to motor theta pairs

        # take some sort of file
        # make xy vector
        #
        #
        #
        #
        # x = [2,5,8]
        # y = [15,15,15]
        # x = [5]
        # y = [15.3]

        # xy = np.transpose(np.array([x,y]))
        xy = xy_array

        # w = 10
        # print(xy)

        # numPts = xy.shape[0]
        numPts = len(xy)
        # print(numPts)

        #######################
        # initialze variables #
        #######################

        # empty c1 and c2
        c1 = np.zeros(numPts)
        c2 = np.zeros(numPts)

        # empty alpha and beta
        alpha1 = np.zeros(numPts)
        alpha2 = np.zeros(numPts)
        beta1 = np.zeros(numPts)
        beta2 = np.zeros(numPts)

        # empty motor angles
        theta = np.zeros([numPts,2])

        #############################
        # compute virtual legs c1,2 #
        #############################
        # print('xy')
        # print(xy)
        # print()

        # print(xy[:,0])
        c1_sqrd = np.square(xy[:,0]) + np.square(xy[:,1])

        c2_sqrd = np.square(xy[:,0]-self.w) + np.square(xy[:,1])

        # print('c^2')
        # print(c1_sqrd)
        # print(c2_sqrd)
        # print()

        ########################
        # compute motor angles #
        ########################

        #  alpha is inner angle
        alpha1 = np.arccos((c1_sqrd + self.w**2 - c2_sqrd)/(2*np.sqrt(c1_sqrd)*self.w))
        alpha2 = np.arccos((c2_sqrd + self.w**2 - c1_sqrd)/(2*np.sqrt(c2_sqrd)*self.w))

        # print('alpha deg')
        # print(alpha1*180/np.pi)
        # print(alpha2*180/np.pi)
        # print()

        # beta is outer angle
        beta1 = np.arccos((self.a1**2 - self.b1**2 + c1_sqrd)/(2*self.a1*np.sqrt(c1_sqrd)))
        beta2 = np.arccos((self.a2**2 - self.b2**2 + c2_sqrd)/(2*self.a2*np.sqrt(c2_sqrd)))

        # print('beta deg')
        # print(beta1*180/np.pi)
        # print(beta2*180/np.pi)
        # print()

        if mode[0] == '+':
            theta[:,0] = alpha1 + beta1
        else:
            theta[:,0] = alpha1 - beta1

        if mode[1] == '+':
            theta[:,1] = alpha2 + beta2
        else:
            theta[:,1] = alpha2 - beta2

        # print('theta deg')
        # print(theta*180/np.pi)
        # print()

        return theta

    def animatePath(self, motorAngles, xy_array, frameDelay, width, save, draw, ghost):
        '''
        input:	motor angles and end effector points
        output:	animation of drawing

        vargs:
            width:  float   line width in plot
            save:   BOOL    true -> save mp4
            draw:   BOOL    true -> enable "pen" trace
            ghost:  BOOL    true -> show full path underlay
        '''

        # numPts = motorAngles.shape[0]
        numPts = len(motorAngles)

        ###############
        # driven arms #
        ###############

        # r1,2 have xy coord of each of the driven arms

        # initialze vectors
        r1 = np.zeros([(numPts),2])
        r2 = np.zeros([(numPts),2])

        # generate x values
        r1[:,0] = self.a1 * np.cos(motorAngles[:,0])
        r2[:,0] = self.w - (self.a2 * np.cos(motorAngles[:,1]))

        # generate y values
        r1[:,1] = self.b1 * np.sin(motorAngles[:,0])
        r2[:,1] = self.b2 * np.sin(motorAngles[:,1])

        # could verify desired xy against points generated with forward kinematics
        # or could just plot with xy that we started with


        fig = plt.figure()
        fig.set_dpi(100)
        fig.set_size_inches(7, 6.5)


        buffer = self.w * 0.1
        ax = plt.axes(xlim=(np.min(r1[:,0])-buffer, np.max(r2[:,0])+buffer),
                        ylim=(0-buffer, np.max(xy_array[:,1])+buffer))

        # machine vertices at first point
        vertices = np.array([
        [0,0],
        [r1[0,0], r1[0,1]],
        [xy_array[0,0], xy_array[0,1]],
        [r2[0,0], r2[0,1]],
        [self.w, 0]
        ])

        patch = patches.Polygon(vertices, edgecolor=[100/255,0,1], linewidth=width, closed=True, fill=False)


        def init():
            # patch.center = (5, 5)
            ax.add_line(patch)

            if ghost == True:
                # plot full path
                plt.plot(xy_array[:,0],xy_array[:,1])

            return patch,

        def animate(i):

            # get vertices of machine and build polygon
            vertices = np.array([
            [0,0],
            [r1[i,0], r1[i,1]],
            [xy_array[i,0], xy_array[i,1]]
            ])

            if draw == True:
                '''
                ##########################
                ##########################
                THIS PART IS HORRIBLY SLOW
                TRY TO MAKE FASTER
                ##########################
                ##########################
                '''
                trace_rev = xy_array[0:i]
                trace_fwd = trace_rev[::-1]

                vertices = np.concatenate((vertices,
                                            trace_fwd,
                                            trace_rev
                                            ))
                # vertices = np.concatenate((vertices, trace_rev))

            close = np.array([
            xy_array[i],
            [r2[i,0], r2[i,1]],
            [self.w, 0]
            ])

            vertices = np.concatenate((vertices, close))

            patch.set_xy(vertices)

            # hm = plt.plot(xy_array[0:i,0],xy_array[0:i,1])

            return patch,

        '''def animate(i):

            # get vertices of machine and build polygon
            vertices = np.array([
            [0,0],
            [r1[i,0], r1[i,1]],
            [xy_array[i,0], xy_array[i,1]],
            [r2[i,0], r2[i,1]],
            [self.w, 0]
            ])
            patch.set_xy(vertices)

            # hm = plt.plot(xy_array[0:i,0],xy_array[0:i,1])

            return patch,'''
        if draw == True:
            anim = animation.FuncAnimation(fig, animate,
                                           init_func=init,
                                           frames=numPts,
                                           interval=frameDelay,
                                           blit=True)
        else:
            plt.plot(xy_array[:,0],xy_array[:,1])


        if save == True:
            # Set up formatting for the movie files
            # Writer = animation.writers['ffmpeg']
            # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
            # writer = animation.FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
            # anim.save("movie.mp4", writer=writer)
            p = 0


        plt.show()

        return

    def motorSteps(self, motorAngles, stepPerRev):
        '''
        input:	motor angles and steps per rev of stepper motor
        output:	motor steps (absolute and relative)
        **note: absolute step is measured from ground axis:
        L motor: anti-CW from +x axis
        R motor: CW from -x axis
        '''

        # motorAngles : nx2 array of motor angles
        # stepPerRev : how many stepper motor steps per rev

        stepPerRad = stepPerRev / (2*np.pi) # (step/rev) / (rad/rev) = (step/rad)

        # this is the absolute step number measured from zero
        # this needs to be converted to relative step from point ot point
        steps_absolute = motorAngles * stepPerRad

        # shift is steps_absolute shifted down to allow us to do
        # relative transformation of (n) - (n-1)
        shift = np.insert(steps_absolute, 0, [0,0], axis=0)
        shift = np.delete(shift, -1, axis=0)

        # subtract previous step from current step (both absolutee)
        # to get relative step between each angle pair
        steps_relative = steps_absolute - shift

        return steps_absolute, steps_relative, shift

    def forwardKinematics(self, motorAngles):
        '''

        '''

        numPts = len(motorAngles)

        def dist(p1,p2):
            dist = np.sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )
            return dist

        def deriv(p1,p2):
            deriv = (p2[1] - p1[1])/(p2[0] - p1[0])
            return deriv

        ##################################
        # initialize system state arrays #
        ##################################

        # C is associated with motor1 (origin)
        # F is associated with motor2 (x axis offset)
        Cx = self.a1*np.cos(motorAngles[:,0])
        Cy = self.a1*np.sin(motorAngles[:,0])
        Fx = self.a2*np.cos(np.pi - np.mod(motorAngles[:,1],2*np.pi)) + self.w
        Fy = self.a2*np.sin(np.pi - np.mod(motorAngles[:,1],2*np.pi))

        v1 = (Cy - Fy)/(Fx - Cx)
        # v2_const = 1/2*(self.b1**2 - self.b2**2 - self.a1**2 + self.a2**2)
        # v2 = v2_const/(Fx-Cx)
        # v2 = (-(Cy**2)+self.b1**2+Fy**2-self.b2**2-Cx**2+Fx**2)/(2*(Fx-Cx))
        v2 = (self.b1**2 - self.b2**2 - self.a1**2 + Fx**2 + Fy**2) / (2*(Fx-Cx))
        v3 = v1**2 + 1
        v4 = 2*v1*v2 - 2*v1*Cx - 2*Cy
        v5 = self.a1**2 - self.b1**2 - 2*v2*Cx + v2**2

        ##############################
        # initialize solution arrays #
        ##############################

        #this will store the positive case of +/- of quadratic soln
        xPos = np.zeros(numPts)
        yPos = np.zeros(numPts)

        #this will store the negative case of +/- of quadratic soln
        xNeg = np.zeros(numPts)
        yNeg = np.zeros(numPts)

        xy_pos = np.zeros([numPts,2])
        xy_neg = np.zeros([numPts,2])
        xy_forward = np.zeros([numPts,2]) # [x,y]

        ####################
        # solve kinematics #
        ####################

        yPos = (-v4 + np.sqrt(v4**2 - 4*v3*v5))/(2*v3)  # + quadratic solution
        yNeg = (-v4 - np.sqrt(v4**2 - 4*v3*v5))/(2*v3)  # - quadratic solution

        xPos = v1*yPos + v2
        xNeg = v1*yNeg + v2

        if yPos[0] > yNeg[0]:
            xy_forward[0] = [xPos[0], yPos[0]]
        else:
            xy_forward[0] = [xNeg[0], yNeg[0]]

        if dist([xPos[1],yPos[1]],xy_forward[0]) < dist([xNeg[1],yNeg[1]],xy_forward[0]):
            xy_forward[1] = [xPos[1], yPos[1]]
        else:
            xy_forward[1] = [xNeg[1], yNeg[1]]

        deriv_pos = np.zeros(1)
        deriv_neg = np.zeros(1)

        for pt in np.arange(2,numPts):

            if dist([xPos[pt],yPos[pt]],xy_forward[pt-1]) < dist([xNeg[pt],yNeg[pt]],xy_forward[pt-1]):
                xy_forward[pt] = [xPos[pt], yPos[pt]]
            else:
                xy_forward[pt] = [xNeg[pt], yNeg[pt]]

            deriv_pos = np.append(deriv_pos, deriv([xPos[pt],yPos[pt]], [xPos[pt-1],yPos[pt-1]]))
            deriv_neg = np.append(deriv_neg, deriv([xNeg[pt],yNeg[pt]], [xNeg[pt-1],yNeg[pt-1]]))

            '''
            old_slope = deriv(xy_forward[pt-1], xy_forward[pt-2]) # slope between previous two
            slope_pos = deriv([xPos[pt],yPos[pt]],xy_forward[pt-1])
            slope_neg = deriv([xNeg[pt],yNeg[pt]],xy_forward[pt-1])
            if np.abs(slope_pos - old_slope) < np.abs(slope_neg - old_slope):
                xy_forward[pt] = [xPos[pt], yPos[pt]]
            else:
                xy_forward[pt] = [xNeg[pt], yNeg[pt]]
            '''



        xy_pos[:,0] = xPos
        xy_pos[:,1] = yPos
        xy_neg[:,0] = xNeg
        xy_neg[:,1] = yNeg

        # print(yPos)
        # print(yNeg)
        # print(xy_forward)
        return xy_forward, xy_pos, xy_neg, deriv_pos, deriv_neg

    def pathError(xy_ref, xy_from_angles):
        '''
        input:
        xy_ref: 		original xy array desired by user
        xy_from_angles:	xy array generated from forward kinematics of motor angles
        output:
        % error between xy points
        '''
        error = (xy_from_angles- xy_ref) / xy_ref
        return error

    def generateSingularites(self):

        '''
        generate singularities
        see jung-hyun choi singularity analysis of 5-bar planar
        for singularity modes (fig2, a->f)

        see fallahi study of the workspace of five-bar closed loop

        '''

        singularities = []
        return singularities

    def singularityCheck(self, xy, tolerance):
        # check to see if singularities are near any points on xy path

        # for every point in path, check against each singularity
        for point in xy:
            for singularity in self.singularities :
                dist = distformula(point, singularity)
                if dist < tolerance:
                    # !!! you might run into a singularity !!!
                    warning = 'singularity detected'
                else:
                    warning = 'no singularitites detected'
        return warning
