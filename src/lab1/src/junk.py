    '''
    Students implement (extra credit)
    Initialize the particles to cover the map for global localization.
    Call this instead of initialize_global() in __init__().
    '''
    def initialize_global_loc(self, scan_topic, laser_ray_step, exclude_max_range_rays, 
                                                max_range_meters, map_msg, car_length):
        print "initialize_global_loc"
        self.state_lock.acquire()

        # Get in-bounds locations
        permissible_x, permissible_y = np.where(self.permissible_region == 1)

        angle_step = 4 # The number of particles at each location, each with different rotation
        num_particles = min(self.particles.shape[0]*50, len(permissible_x) * angle_step)
        num_locations = num_particles / angle_step
        permissible_step = len(permissible_x)/num_locations # The sample interval for permissible states
        indices = np.arange(0, len(permissible_x), permissible_step)[:num_locations] # Indices of permissible states to use
        permissible_states = np.zeros((num_particles,3)) # Proxy for the new particles
        permissible_weights = np.full(num_particles, 1.0 / num_particles)

        # Loop through permissible states, each iteration drawing particles with
        # different rotation
        for i in xrange(angle_step):
            permissible_states[i*num_locations:(i+1)*num_locations,0] = permissible_y[indices]
            permissible_states[i*num_locations:(i+1)*num_locations,1] = permissible_x[indices]
            permissible_states[i*num_locations:(i+1)*num_locations,2] = i*(2*np.pi / angle_step)
        # Transform permissible states to be w.r.t world 
        Utils.map_to_world(permissible_states, self.map_info)

        # TODO: Weight permissible_states by sensor model
        # 
        # pseudocode:
        # foreach states_i in permissible_states:
        #     expected_scan = perform ray casting
        #     likelihood = prob of scan given expected_scan
        #     weights of states_i = likelihood
        # normalize weights to be a probability
        self.publish_particles(permissible_states)

        self.gl_sensor_model = GlobalLocSensorModel(scan_topic, laser_ray_step, exclude_max_range_rays, max_range_meters, map_msg,
                        permissible_states, permissible_weights, car_length, self.particles, self.weights, update_times=20, _pf=self)

        self.state_lock.release()
