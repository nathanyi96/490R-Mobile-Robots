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


  def initialize_global_random(self):
    self.state_lock.acquire()
    # Get in-bounds locations
    permissible_x, permissible_y = np.where(self.permissible_region == 1)
    
    
    angle_step = 4 # The number of particles at each location, each with different rotation
    num_locations = self.particles.shape[0] / angle_step
    indices = np.random.choice(range(len(permissible_x)), size=num_locations, replace=False) # Indices of permissible states to use
    permissible_states = np.zeros((self.particles.shape[0],3)) # Proxy for the new particles
    
    # Loop through permissible states, each iteration drawing particles with
    # different rotation
    for i in xrange(angle_step):
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),0] = permissible_y[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),1] = permissible_x[indices]
      permissible_states[i*(self.particles.shape[0]/angle_step):(i+1)*(self.particles.shape[0]/angle_step),2] = i*(2*np.pi / angle_step) 
     
    # Transform permissible states to be w.r.t world 
    Utils.map_to_world(permissible_states, self.map_info)
    
    # Reset particles and weights
    self.particles[:,:] = permissible_states[:,:]
    self.weights[:] = 1.0 / self.particles.shape[0]
    self.publish_particles(self.particles)
    self.state_lock.release()


  def initialize_global_loc2(self):

    self.global_localize_cnt +=  1
    # Reset particles and weights
    self.state_lock.acquire() 
    angle_step = 4  # The number of particles at each location, each with different rotation
    radius_factor = 1.7
    print 'initialize_global_loc2'

    def upsample(state, radius, num):
        '''
        Sample and return num particles around state, within a l-inf radius of d.
        state : 1x3 ndarray - the state to resample around
        radius: float  - l-inf radius of the neighborhood (in units of state)
        num   : int   - number of particles sampled around the neighborhood of state
        '''
        original_state = state.copy()
        Utils.world_to_map(state, self.map_info)

        #print '--------- this is a state', state
        x, y, theta =  int(state[0, 0]), int(state[0, 1]), state[0, 2]
        x_start = max(x-radius, 0)
        y_start = max(y-radius, 0)
        x_end = min(x+radius, self.map_info.height)
        y_end = min(y+radius, self.map_info.width)
        state_offset = np.array([[x_start, y_start, 0]])
        #print self.permissible_region.shape, self.map_info.height, self.map_info.width, self.map_info.resolution

        num_locations = num / angle_step
        permissible_x, permissible_y = np.where(self.permissible_region[y_start:y_end, x_start:x_end] == 1)
        if len(permissible_x) < num_locations or num_locations == 0:
            return original_state
        permissible_step = len(permissible_x) / num_locations
        print 'check', num_locations, len(permissible_x), permissible_step
        indices = np.arange(0, len(permissible_x), permissible_step)[:num_locations] # Indices of permissible states to use
        permissible_states = np.zeros((num, 3)) # Proxy for the new particles
        angle_step_ratio = 2 * np.pi #radius / self.map_info.height * 2 * np.pi 
        for i in xrange(int(angle_step)):
            permissible_states[i*num_locations:(i+1)*num_locations,0] = permissible_y[indices]
            permissible_states[i*num_locations:(i+1)*num_locations,1] = permissible_x[indices]
            permissible_states[i*num_locations:(i+1)*num_locations,2] = theta + ((i - angle_step / 2) * angle_step_ratio) # i*(2*np.pi / angle_step) 
     
        # Transform permissible states to be w.r.t world
        permissible_states += state_offset
        Utils.map_to_world(permissible_states, self.map_info)
        permissible_states[-1] = original_state
        return permissible_states

    particles_num = self.particles.shape[0]
    radius = int(min(self.map_info.height, self.map_info.width) / (radius_factor ** self.global_localize_cnt)); # decrease radius and selection number
    print 'current radius', radius
    particle_candidate_num = 40;  
    current_particles = self.particles[:]
    # sample
    # candidate_indices = np.random.choice(self.particle_indices, particle_candidate_num, p=self.weights) # use top 3
    # candidates = current_particles[candidate_indices]
    # top k
    if self.global_localize_cnt < 5:
        candidate_indices = np.argsort(self.weights)[-particle_candidate_num:]
        candidates = current_particles[candidate_indices]
        upsample_candidate = [int(particles_num / particle_candidate_num)] * candidates.shape[0]
    # low variance
    else:
        self.state_lock.release()  
        self.resampler.resample_low_variance()
        self.state_lock.acquire() 
        candidates, counts = np.unique(self.particles, axis=0, return_counts=True)
        upsample_candidate = counts
    new_particles = []
    for idx in range(candidates.shape[0]):
        candidate = candidates[[idx]]
        if upsample_candidate[idx] == 1:
            candidates_upsample = candidate # keep this bad sample
        else:    
            candidates_upsample = upsample(candidate, radius, upsample_candidate[idx])
        new_particles.append(candidates_upsample)
    new_particles = np.concatenate(new_particles, axis=0)
    print 'total upsample shape', new_particles.shape[0]
    particles_num = new_particles.shape[0]
    current_particles[:particles_num] = new_particles[:particles_num] # more fine grained particle
    if particles_num == 0:
        self.global_localize = False    
    self.weights[:] = 1.0 / self.particles.shape[0]  
    
    if self.global_localize_cnt > 8: # finish and return to normal
        self.global_localize = False    
    self.state_lock.release()


  def noisy_update(self):
    LOCATION_NOISE = 0.5
    HEADING_NOISE = 0.5
    ENTROPY_THRES = 2
    # print "--------------=---- loc 3"
    # print "--------------=---- loc 3 ---2"
    self.state_lock.acquire()
    ent = -((self.weights*np.log2(self.weights)).sum())
    print 'entropy ==', ent
    self.state_lock.release()
    
    #if ent < ENTROPY_THRES:
    self.resampler.resample_low_variance()

    self.state_lock.acquire()
    #print exp_weights.shape, "------------"
    exp_weights = 3*np.exp(-3*self.weights)
    new_particles = self.particles.copy()
    new_particles[:,0] += np.random.normal(0, LOCATION_NOISE*exp_weights, size=self.particles.shape[0])
    new_particles[:,1] += np.random.normal(0, LOCATION_NOISE*exp_weights, size=self.particles.shape[0])

    # filter points outside of permissible region
    Utils.world_to_map(new_particles, self.map_info)
    permissible_particles = []
    for i in xrange(new_particles.shape[0]):
        x, y = new_particles[i,0], new_particles[i,1]
        if x < 0: x = 0
        if y < 0: y = 0
        if x >= self.map_info.width: x = self.map_info.width-1
        if y >= self.map_info.height: y = self.map_info.height-1
        if self.permissible_region[int(y), int(x)]:
            permissible_particles.append(new_particles[i,:])
    permissible_particles = np.array(permissible_particles)
    Utils.map_to_world(permissible_particles, self.map_info)
    npb = permissible_particles.shape[0]

    supp_indices = np.random.choice(range(npb), size=self.particles.shape[0]-npb, replace=True)
    self.particles[:npb,:2] = permissible_particles[:,:2]
    self.particles[npb:,:2] = permissible_particles[supp_indices,:2]

    self.particles[:,2] += np.random.normal(0, HEADING_NOISE*exp_weights, size=self.particles.shape[0])
    self.particles[self.particles[:,2] < -1*np.pi,2] += 2*np.pi
    self.particles[self.particles[:,2] > np.pi,2 ]-= 2*np.pi

    self.state_lock.release()
    self.noisy_cnt += 1
    if self.noisy_cnt >= 10:
        self.global_suspend = False
