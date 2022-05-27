
# training gaussian mixture model 
from math import fabs
from sklearn.mixture import BayesianGaussianMixture
import numpy as np
    
def view_pose_generation(obj, target):
    target_particle = obj[target]
    target_particle.resampling()

    #GMM
    #target_particle.p.x is n x 3
    bgm = BayesianGaussianMixture(n_components = 10, covariance_type="full").fit(target_particle.p.x)
    g_mean = bgm.means_
    g_weight = bgm.weights_
    g_covariance = bgm.covariances_

    
    #tau_ are all (num_candidate_poses, num_col)
    return bgm, tau_weight, tau_translation, tau_rotation



def view_pose_selection(obj, bgm, tau_weight, tau_translation, tau_rotation, target, landmark):
    # Hybrid Search

    alpha = 0.1
    beta = 0.4
    sigma = 0.5

    util = np.zeros(np.shape(tau_weight)[0])
    for i in range(np.shape(tau_weight)[0]):
        util+=tau_weight[i]
        d_nav = A_star(tau_translation, target) # A_star algorithm can be computed in some way
        util+=alpha/tanh(sigma*d_nav)
        

        util_landmark = np.zeros(len(landmark), np.shape(tau_weight)[0])
        for j in range(len(landmark)):
            for n in range(np.shape(tau_weight)[0]):
                B_t_j = belief(target, landmark[j])
                CoOcurr_t_j = 1 - B_t_j['Disjoint']
                
                #belief of location of j
                bel_loc = np.average(obj[landmark[j]].p.x, weights=obj[landmark[j]].p.w)

                w_j_n = bgm.weights_[bgm.predict(bel_loc)]
                
                flag = 0
                if bgm.means_[n] in E_tau:
                    flag = 1
                util_landmark[j][n] = CoOcurr_t_j * w_j_n * flag
        
        sorted_util_landmark = util_landmark.flatten().sort()
        max_util_landmark = sorted_util_landmark[-1]

        util+=beta * max_util_landmark
    
    max_index = np.argmax(util)

    return tau_translation[max_index], tau_rotation[max_index]
