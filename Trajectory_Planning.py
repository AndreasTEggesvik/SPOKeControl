import numpy as np

# constraints = [q(t0), d(q(t0)), d(q(tf))], t = [t0, tf]
# example: ([0.2, 0, 5], [0, 20])
def calculateTrajectory(constraints, t):
    
    # constraints = [q(t0), d(q(t0)), d(q(tf))]
    if len(constraints) == 3:
        M = np.array([[1, t[0], t[0]**2],
                    [0, 1, 2*t[0]],
                    [0, 1, 2*t[1]]])

    # constraints = [q(t0), d(q(t0)), q(tf), d(q(tf))]
    elif len(constraints) == 4:
        M = np.array([[1, t[0], t[0]**2, t[0]**3],
                    [0, 1, 2*t[0], 3*t[0]**2],
                    [1, t[1], t[1]**2, t[1]**3],
                    [0, 1, 2*t[1], 3*t[1]**2]])
                    
    b = np.array(constraints)
    M_inv = np.linalg.inv(M)
    
    a = M_inv.dot(b)
    return a

#######################################################################################################
# Helper function that calculates the desired constant velocity based on desired distance and time.   #
# mergingValue in range [0-1]                                                                         #
# Quick acceleration for low return value, slow acceleration for high return value                    #
#######################################################################################################
def getLSPB_velocity(q0, qf, t0, tf, mergingValue):
    return abs((1 + mergingValue) * (qf-q0)/(tf-t0))


#########################################################################
# Calculates a LSPB given a set of constraints.                         #
# The trajectory is given by three matricies containing the constants   #
#   for a polynomial function describing position over time             #
# constraints = [q(t0), d(q(t0)), q(tf), d(q(tf))], t = [t0, tf]        #
#   example on set of valid arguments (0.2, [0, 0, 2, 0], [0, 5])       #
# LSPB = linear segment with parabolic blends                           #
#########################################################################
def LSPB(V, constraints, t):
    [q0, dq0, qf, dqf] = constraints
    [t0, tf] = t
    
    # Validity test
    if (not (qf - q0)/V < tf-t0):
        print("Too small tf relative to V")
        return False
    elif (not tf-t0 < 2*(qf - q0)/V):
        print("Too large tf relative to V")
        return False
    
    tb = (q0 - qf + V*tf)/V
    
    A0 = calculateTrajectory([q0, dq0, V], [t0, tb])
    A1 = np.array([(q0 + qf - V*tf)/2, V])
    A2 = np.array([qf - A0[2]*tf**2, 2*A0[2]*tf, -A0[2]])
    return [A0, A1, A2, tb]


####################################################################
# Calculates the desired poisition of a trajectory at a given time #
# A is a vector                                                    #
# t is integer                                                     #
####################################################################
def getTrajectoryPosition(A, t):
    if (A.shape[0] == 2):
        timeVector = np.array([1, t])
    elif (A.shape[0] == 3):
        timeVector = np.array([1, t, t**2])
    elif (A.shape[0] == 4):
        timeVector = np.array([1, t, t**2, t**3])
    else:
        return False
    return A.dot(timeVector)

#########################################################
# Calculates the desired position for a LSPB trajectory #
#   at a given time                                     #
#########################################################
def getLSPB_position(A0, A1, A2, t0, tb, tf, t):
    if (t < 0):
        return False
    elif (t < t0):
        return getTrajectoryPosition(A0, t0)
    elif (t0 <= t <= tb):
        return getTrajectoryPosition(A0, t)
    elif (tb < t < tf - tb):
        return getTrajectoryPosition(A1, t)
    elif (tf - tb <= t <= tf):
        return getTrajectoryPosition(A2, t)
    elif (t> tf):
        return getTrajectoryPosition(A2, tf)





















