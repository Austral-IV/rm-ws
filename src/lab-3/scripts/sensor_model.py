#!/usr/bin/env python3
from numbers import Integral
import numpy as np
from scipy.ndimage.filters import convolve
from filterpy.discrete_bayes import predict, update, normalize
map_dim = (10,10) # Dimensiones del mapa
belief0 = 1 / (map_dim[0]*map_dim[1]) * np.ones(map_dim) # prior: inicialmente hay prob uniforme en todo el mapa
likelyhood = 1 / (map_dim[0]*map_dim[1]) * np.ones(map_dim) # prior: inicialmente hay prob uniforme en todo el mapa
belief0[3,3] = 1
belief0 = normalize(belief0)

#------------------------ WIP-----------------------------------


resolution=0.01
zmax = 4
distances = int(zmax/resolution)
prior = np.ones(distances)/distances
def laser_range_filter(prior, ranges, range_accuracy = 0.9):
    X = len(prior)
    T = len(ranges)
    Z = np.ones(X)*range_accuracy
    px_u = np.zeros(X)

    for t in range(T):
        #no hay movimiento, por lo que p(x|u,xt-1) = 1
        bel_bar = np.sum(prior)
        # hacer tq pz_x
        #nu = 1/range_acc*la distancia medida + (1-r_acc*el resto)
        nu = 1/(range_accuracy*prior(range(t)) + np.sum((1-range_accuracy)*prior) 
            - (1-range_accuracy)*prior(range(t)))

        belt = nu*Z*bel_bar
        prior = belt

    return belt
