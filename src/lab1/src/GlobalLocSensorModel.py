import numpy as np
from SensorModel import SensorModel
from ReSample import ReSampler
import time

class GlobalLocSensorModel(SensorModel):

    def __init__(self, scan_topic, laser_ray_step, exclude_max_range_rays, 
               max_range_meters, map_msg, particles, weights, car_length, real_particles, real_weights, state_lock=None, update_times=1, _pf=None):
        super(GlobalLocSensorModel, self).__init__(scan_topic, laser_ray_step, exclude_max_range_rays,
               max_range_meters, map_msg, particles, weights, car_length, state_lock)
        self.real_particles = real_particles
        self.real_weights = real_weights
        self.resampler = ReSampler(self.particles, self.weights, self.state_lock)
        self.update_count = 0
        self.UPDATE_LIMIT = update_times
        self.alive = True
        self.RESAMPLE_P = 5
        self.resample_cnt = 0
        self._pf = _pf

    def lidar_cb(self, msg):
        super(GlobalLocSensorModel, self).lidar_cb(msg)
        time.sleep(1.0)
        self.resample_cnt += 1
        self.update_count += 1
        if self.resample_cnt == self.RESAMPLE_P:
            self.resample_cnt = 0
            self.resampler.resample_low_variance()

        if self._pf:
            self._pf.visualize()
        
        if self.update_count == self.UPDATE_LIMIT:
            self.laser_sub.unregister()
            self.alive = False
            indices = np.random.choice(range(self.particles.shape[0]), size=self.real_particles.shape[0], replace=True, p=self.weights)
            particles = self.particles[indices] # I M GOING TO LAB  ok? yes
    
            # Reset particles and weights
            self.real_particles[:,:] = particles[:,:]
            self.real_weights[:] = 1.0 / self.real_particles.shape[0]
            print "global localization complete"
 
