import numpy as np
import halton as h
import time

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

        start_time = time.time()
        n_headings = 4
        step = 2*np.pi / n_headings
        n_xy = (num_samples-1) // n_headings + 1
        samples = np.empty((n_xy * n_headings, 3))
        xy_samples = self.sample_valid_xy_halton(n_xy)
        x_samples, y_samples = xy_samples[:,0], xy_samples[:,1]
        for i in range(n_headings):
            samples[i*n_xy:(i+1)*n_xy,0] = x_samples
            samples[i*n_xy:(i+1)*n_xy,1] = y_samples
            samples[i*n_xy:(i+1)*n_xy,2] = (i * step) + np.random.normal(0, 0.03, size=n_xy)
        print 'sample time:', time.time()-start_time

        print 'sampling range:', self.xlimit, self.ylimit

        #x_samples = np.random.uniform(self.xlimit[0], self.xlimit[1], num_samples)
        #y_samples = np.random.uniform(self.ylimit[0], self.ylimit[1], num_samples)
        #x_samples = np.random.randint(self.xlimit[0], self.xlimit[1], num_samples)
        #y_samples = np.random.randint(self.ylimit[0], self.ylimit[1], num_samples)
        
        # xy_samples = self.sample_valid_xy(num_samples)
        # theta_samples = np.random.uniform(0, 2*np.pi, num_samples)
        # samples = np.stack((xy_samples[:,0], xy_samples[:,1], theta_samples), axis=1)
        
        #samples = np.stack((x_samples, y_samples, theta_samples), axis=1)
        # samples[:,0] -= 0.5
        # samples[:,1] -= 0.5

        # Implement here
        return samples


    def sample_valid_xy(self, num_samples):
        cnt = 0
        samples = np.empty((num_samples, 2))
        while cnt < num_samples:
            xs = np.random.uniform(self.xlimit[0], self.xlimit[1], num_samples-cnt)
            ys = np.random.uniform(self.ylimit[0], self.ylimit[1], num_samples-cnt)
            subsamples = np.stack((xs, ys), axis=1)
            valids = self.env.state_validity_checker(subsamples)
            end_cnt = cnt+np.count_nonzero(valids)
            samples[cnt:end_cnt] = subsamples[valids]
            cnt = end_cnt
        return samples


    def sample_valid_xy_halton(self, num_samples):
        SAMPLING_FACTOR = 5
        samples = np.array(h.halton(2, SAMPLING_FACTOR*num_samples))
        start = np.array([[self.xlimit[0], self.ylimit[0]]])
        end = np.array([[self.xlimit[1], self.ylimit[1]]])
        samples = start + (end - start - 1) * samples

        valids = self.env.state_validity_checker(samples)
        final_samples = samples[valids]
        assert final_samples.shape[0] >= num_samples
        return final_samples[:num_samples]
        



