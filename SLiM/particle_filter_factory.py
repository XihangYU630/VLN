
import numpy as np
from numpy.random import randn
from inference import particle_filter
from search import view_pose_generation, view_pose_selection
import matplotlib.pyplot as plt
import math


class myStruct:
    pass

# process model
def process_model(x, w):
    f = np.array([x[0], x[1]]).reshape([2, 1]) + w
    return f.reshape([2, 1])

# measurement model
def measurement_model(x):
    h = np.array([[np.sqrt(x[0] ** 2 + x[1] ** 2)], [math.atan2(x[0], x[1])]])
    return h.reshape([2, 1])
# localizing multiple objects using particle filter

if __name__ == '__main__':
    
    target = 0 # target object is hard coded
    z = np.zeros([3, len(100)]) # measurements are hard coded for now

    # build the system
    sys = myStruct()
    sys.f = process_model
    sys.h = measurement_model
    sys.num_obj = 100 # hard coded for now 

    x = np.empty([3, np.shape(z)[1]])  # state
    x[:, 0] = [np.nan, np.nan]

    obj = []
    for j in range(sys.num_obj):
        # initialization
        init = myStruct()
        init.M = 100
        init.x = np.zeros([3, 1])
        init.Sigma = 1 * np.eye(3)

        obj.append(particle_filter(sys, init))

    tau = view_pose_generation(obj)
    z[0] = view_pose_selection(obj, tau)


    # main loop; iterate over the measurements z
    for k in range(1, np.shape(z)[1], 1):
        # iterate over all objects
        for i in range(sys.num_obj):
            obj[i].resampling()
            obj[i].sample_motion()
            obj[i].importance_measurement(z[:, k].reshape([2, 1]), obj, i)
            

            wtot = np.sum(obj[i].p.w)
            if wtot > 0:
                a = obj[i].p.x
                b = obj[i].p.w
                x[0, k] = np.sum(obj[i].p.x[:, 0] * obj[i].p.w.reshape(-1)) / wtot
                x[1, k] = np.sum(obj[i].p.x[:, 1] * obj[i].p.w.reshape(-1)) / wtot
                x[2, k] = np.sum(obj[i].p.x[:, 2] * obj[i].p.w.reshape(-1)) / wtot
            else:
                print('\033[91mWarning: Total weight is zero or nan!\033[0m')
                x[:, k] = [np.nan, np.nan]
        
        bgm, tau_weight, tau_translation, tau_rotation = view_pose_generation(obj, target)
        z[k] = view_pose_selection(obj, bgm, tau_weight, tau_translation, tau_rotation, target)


