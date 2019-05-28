import numpy as np
import halton as h

class DubinsSampler:
    def __init__(self, env):
        self.env = env
        self.xlimit = self.env.xlimit
        self.ylimit = self.env.ylimit

    def sample(self, num_samples):
        """
        Samples configurations.
        Each configuration is (x, y, angle).

        @param num_samples: Number of sample configurations to return
        @return 2D numpy array of size [num_samples x 3]
        """

        #x_samples = np.random.uniform(self.xlimit[0], self.xlimit[1], num_samples)
        #y_samples = np.random.uniform(self.ylimit[0], self.ylimit[1], num_samples)
        x_samples = np.random.randint(self.xlimit[0], self.xlimit[1], num_samples)
        y_samples = np.random.randint(self.ylimit[0], self.ylimit[1], num_samples)
        theta_samples = np.random.uniform(0, 2*np.pi, num_samples)
        samples = np.stack((x_samples, y_samples, theta_samples), axis=1)

        # samples = np.array(h.halton(3, num_samples))
        # start = np.array([[self.xlimit[0], self.ylimit[0], 0]])
        # end = np.array([[self.xlimit[1], self.ylimit[1], 2 * np.pi]])
        # samples = start + (end - start - 1) * samples
        # samples = samples.astype(np.int)
        # Implement here
        return samples
