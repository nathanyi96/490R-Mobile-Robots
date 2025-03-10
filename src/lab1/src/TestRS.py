#Testing Code for ReSample.py
#Matt Schmittle
#DO NOT EDIT

import matplotlib.pyplot as plt
from ReSample import ReSampler
import numpy as np

if __name__ == '__main__':

  n_particles = 100# The number of particles    
  k_val = 50# Number of particles that have non-zero weight
  trials = 10# The number of re-samplings to do
  num_figs = 3 #number of figures to make
  
for f in range(0,num_figs):
  k_val = (f + 1)*30
  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  for i in xrange(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)
    
    rs = ReSampler(particles, weights) # Create the Resampler
    rs.resample_low_variance()
  
    # Add the number times each particle was sampled    
    for j in xrange(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1
    
  # Display as histogram
  plt.figure('k = '+str(k_val))
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
plt.show()

