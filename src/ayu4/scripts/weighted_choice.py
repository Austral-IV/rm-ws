import numpy as np


if __name__ == '__main__':

  particles = [[0, 0], [1, 1], [2, 2], [3, 3,]]
  weights = [0.02, 0.02, 0.94, 0.02]
  indices = range( len( particles ) )
  samples = np.random.choice( indices, 4, p = weights )
  print( 'sampled particles: %s' % (str([particles[s] for s in samples])) )


