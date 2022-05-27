
# training gaussian mixture model 
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



def view_pose_selection(obj, bgm, tau_weight, tau_translation, tau_rotation, target):
    # Hybrid Search

    alpha = 0.1
    beta = 0.4
    sigma = 0.5

    util = np.zeros(np.shape(tau_weight)[1])
    for i in range(np.shape(tau_weight)[1]):
        util+=tau_weight[i]
        d_nav = A_star(tau_translation, target) # A_star algorithm can be computed in some way
        util+=alpha/tanh(sigma*d_nav)

        # we have landmarks