import numpy as np
import IPython
import halton as h

class Sampler:
    def __init__(self, env):
        self.env = env
        self.xlimit = self.env.xlimit
        self.ylimit = self.env.ylimit

    def sample(self, num_samples, use_random=False):
        """
        Samples configurations.
        Each configuration is (x, y).

        @param num_samples: Number of sample configurations to return
        @return 2D numpy array of size [num_samples x 2]
        """

        # Implement here
        # if use_random:
        #     x = np.random.uniform(self.xlimit[0], self.xlimit[1], num_samples)
        #     y = np.random.uniform(self.ylimit[0], self.ylimit[1], num_samples)
        #     x = np.random.randint(self.xlimit[0], self.xlimit[1], num_samples)
        #     y = np.random.randint(self.ylimit[0], self.ylimit[1], num_samples)
        #     samples = np.stack((x, y), axis=1)
        # else:  # using halton sequence
        #     samples = np.array(halton_sequence(num_samples, 2)).T
        #     samples[:,0] *= (self.xlimit[1] - self.xlimit[0])
        #     samples[:,1] *= (self.ylimit[1] - self.ylimit[0])
        #     samples[:,0] += self.xlimit[0]
        #     samples[:,1] += self.ylimit[0]

        SAMPLING_FACTOR = 2
        samples = np.array(h.halton(2, SAMPLING_FACTOR*num_samples))
        start = np.array([[self.xlimit[0], self.ylimit[0]]])
        end = np.array([[self.xlimit[1], self.ylimit[1]]])
        samples = start + (end - start - 1) * samples

        valids = self.env.state_validity_checker(samples)
        final_samples = samples[valids]
        assert final_samples.shape[0] >= num_samples
        return final_samples[:num_samples]

        # samples = np.array(h.halton(2, num_samples))
        # start = np.array([[self.xlimit[0], self.ylimit[0]]])
        # end = np.array([[self.xlimit[1], self.ylimit[1]]])
        # samples = start + (end - start - 1) * samples

        return samples

# code copied and modified from https://laszukdawid.com/2017/02/04/halton-sequence-in-python/
def vdc(n, base=2):
    vdc, denom = 0, 1
    while n:
        denom *= base
        n, remainder = divmod(n, base)
        vdc += remainder/float(denom)
    return vdc


def halton_sequence(size, dim):
    seq = []
    primeGen = next_prime()
    next(primeGen)
    for d in xrange(dim):
        base = next(primeGen)
        seq.append([vdc(i, base) for i in xrange(size)])
    return seq


def next_prime():
    def is_prime(num):
        """Checks if num is a prime value"""
        for i in range(2,int(num**0.5)+1):
            if(num % i)==0: return False
        return True
    prime = 3
    while(1):
        if is_prime(prime):
            yield prime
        prime += 2


