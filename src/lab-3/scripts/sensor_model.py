#!/usr/bin/env python3
import numpy as np
from scipy.ndimage.filters import convolve
from filterpy.discrete_bayes import predict, update, normalize
map_dim = (10,10) # Dimensiones del mapa
belief0 = 1 / (map_dim[0]*map_dim[1]) * np.ones(map_dim) # prior: inicialmente hay prob uniforme en todo el mapa
likelyhood = 1 / (map_dim[0]*map_dim[1]) * np.ones(map_dim) # prior: inicialmente hay prob uniforme en todo el mapa
belief0[3,3] = 1
belief0 = normalize(belief0)
# def bayes_filter



def gkern(l=5, sig=1.):
    """https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    creates gaussian kernel with side length `l` and a sigma of `sig`
    """
    ax = np.linspace(-(l - 1) / 2., (l - 1) / 2., l)
    gauss = np.exp(-0.5 * np.square(ax) / np.square(sig))
    kernel = np.outer(gauss, gauss)
    return kernel / np.sum(kernel)



def predict_move(prior, movement, kernel):
    """
    Predice el estado del mapa a partir del estado actual y el movimiento.
    El kernel indica la relación re prob over/undershoot
    """
    return convolve(np.roll(prior, movement), kernel, mode='wrap')



# asumimos que el movimiento tiene una distribución gausiana
mov_kernel = gkern(l=3, sig = 0.5)
prior = predict(belief0, [1,1], mov_kernel, mode = 'wrap')
posterior = update(likelyhood, prior)
belief0 = posterior.round(1) ; belief0[np.isnan(posterior)] = 0 ; belief0[posterior < 0] = 0 ; belief0



