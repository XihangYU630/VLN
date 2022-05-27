
from os import P_PIDFD
import numpy as np
from numpy.random import randn, rand

def pi(o_i_t):
    #TODO



def compute_phi_m(o_i_t):
    
    # z_i_t is related to view selection given pose at time t

    #E_i_t can be computed through ai2thor api
    #z_i_t can be computed through ai2thor api

    if o_i_t in E_i_t and z_i_t == None:
        phi_m = P_FN
    elif o_i_t not in E_i_t and z_i_t == None:
        phi_m = P_TN
    elif pi(o_i_t) in z_i_t:
        phi_m = P_TP
    else:
        phi_m = P_FP

    return phi_m

def compute_phi_c_r(obj_i, obj_j, R_i_j):
    if R_i_j == 'In' or R_i_j == 'On' or R_i_j == 'Contain' or R_i_j == 'Support':
        if no_collision(obj_i,obj_j):
            phi_c_r = 1
        else:
            phi_c_r = 0
    elif R_i_j == 'Proximity':
        Cov_i_j = 1 # 
        Cov_i_j = tanh(abs(get_size(obj_i)), abs(get_size(obj_j))) 
        # get_size is a build-in function
        # which extracts the size info of objects
        phi_c_r = np.random.normal(obj_i, Cov_i_j, 1)
    else : # R_i_j == 'Disjoint'
        phi_c_r = 1 
        phi_c_r -= compute_phi_c_r(obj_i, obj_j, 'In') 
        phi_c_r -= compute_phi_c_r(obj_i, obj_j, 'On')
        phi_c_r -= compute_phi_c_r(obj_i, obj_j, 'Contain')
        phi_c_r -= compute_phi_c_r(obj_i, obj_j, 'Support')
        phi_c_r -= compute_phi_c_r(obj_i, obj_j, 'Proximity')
    return phi_c_r


def compute_belief(R_i_j):
    belief = 0.2 # this is hard coded for now
    # actually the paper uses factor graph to compute belief 
    # considering both common sense and lexical consistency
    return belief


def compute_sum_spatial_relation(obj_i, obj_j):
    
    r = ['In', 'On', 'Contain', 'Support', 'Proximity', 'Disjoint']

    phi_c_B = 0

    for R_i_j in r:
        for i in range(obj_j.M):

            phi_c_r = compute_phi_c_r(obj_i, obj_j, R_i_j)
            belief = compute_belief(R_i_j)
            
            phi_c_B += belief * obj_j.p.w[i] * phi_c_r

   return phi_c_B

class particle_state():
    def __init__(self):
        self.x = []
        self.w = []


# This function is used to wrap angles in radians to the interval [-pi, pi]
# pi maps to pi and -pi maps to -pi
def wrapToPI(phase):
    x_wrap = np.remainder(phase, 2 * np.pi)
    while abs(x_wrap) > np.pi:
        x_wrap -= 2 * np.pi * np.sign(x_wrap)
    return x_wrap

# Particle filter class for state estimation of a nonlinear system
# The implementation follows the Sample Importance Resampling (SIR)
# filter a.k.a bootstrap filter
class particle_filter:

    def __init__(self, system, init):
        # Particle filter construct an instance of this class
        #
        # Input:
        #   system: system and noise models
        #   init:   initialization parameters

        self.f = system.f  # process model
        self.h = system.h  # measurement model
        self.M = init.M  # number of particles

        # initialize particles
        self.p = particle_state()  # particles

        wu = 1 / self.M  # uniform weights
        L_init = np.linalg.cholesky(init.Sigma)
        for i in range(self.M):
            self.p.x.append(np.dot(L_init, randn(len(init.x), 1)) + init.x)
            self.p.w.append(wu)
        self.p.x = np.array(self.p.x).reshape(-1, len(init.x))
        self.p.w = np.array(self.p.w).reshape(-1, 1)

    def sample_motion(self):
        #TODO

    def importance_measurement(self, z, obj, i):
        # compare important weight for each particle based on the obtain range and bearing measurements
        #
        # Inputs:
        #   z: measurement
        #   
        w = np.zeros([self.M, 1])  # importance weights
        for i in range(self.M):
            # compute innovation statistics

            phi_m = compute_phi_m(self.p.x[i])

            product_phi_c_B = 1
            for obj_j, j in enumerate(obj):
                if j != i:
                    sum_spatial_relation = compute_sum_spatial_relation(self, obj_j)
                    product_phi_c_B *= sum_spatial_relation

            self.p.w[i] = phi_m * product_phi_c_B

        # normalize weights
        self.p.w = self.p.w / np.sum(self.p.w)
        # compute effective number of particles
        # self.Neff = 1 / np.sum(np.power(self.p.w, 2))  # effective number of particles

    def resampling(self):
        # low variance resampling
        W = np.cumsum(self.p.w)
        r = rand(1) / self.Mn
        # r = 0.5 / self.M
        j = 1
        for i in range(self.M):
            u = r + (i - 1) / self.M
            while u > W[j]:
                j = j + 1
            self.p.x[i, :] = self.p.x[j, :]
            self.p.w[i] = 1 / self.M






