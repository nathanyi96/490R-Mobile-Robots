import numpy as np

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

        # Implement here
        return samples
