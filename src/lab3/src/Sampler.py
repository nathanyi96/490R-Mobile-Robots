import numpy as np
import IPython
import halton as h

class Sampler:
    def __init__(self, env):
        self.env = env
        self.xlimit = self.env.xlimit
        self.ylimit = self.env.ylimit

    def sample(self, num_samples):
        """
        Samples configurations.
        Each configuration is (x, y).

        @param num_samples: Number of sample configurations to return
        @return 2D numpy array of size [num_samples x 2]
        """

        # Implement here
        #x = np.random.uniform(self.xlimit[0], self.xlimit[1], num_samples)
        #y = np.random.uniform(self.ylimit[0], self.ylimit[1], num_samples)
        x = np.random.randint(self.xlimit[0], self.xlimit[1], num_samples)
        y = np.random.randint(self.ylimit[0], self.ylimit[1], num_samples)
        samples = np.stack((x, y), axis=1)
        '''
        samples = np.array(h.halton(2, num_samples)).T

        '''

        return samples
