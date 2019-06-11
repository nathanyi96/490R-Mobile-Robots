#!/usr/bin/python

import util
import numpy as np
import rospy
from nav_msgs.srv import GetMap
from lab2.srv import ReadFile

class WaypointSampler(object):
    ''' Sample waypoints to follow'''
    def __init__(self):
      rospy.init_node('waypoints_sampler', anonymous=True)
      self.mapdata = np.load('../../lab3/src/MapData.npy').T
      self.map_info = self.get_map().info
      self.xlimit = (1767, 2223)
      self.ylimit = (2312, 2793)

    def sample_free(self):
      '''map frame single free sampler'''
      x = np.random.randint(self.xlimit[0], self.xlimit[1])
      y = np.random.randint(self.ylimit[0], self.ylimit[1])
      while(self.strict_checker(x,y,5)):
        x = np.random.randint(self.xlimit[0], self.xlimit[1])
        y = np.random.randint(self.ylimit[0], self.ylimit[1])
      score = np.random.randint(1,11)
      sample = np.array([x,y,0], dtype=np.float64)
      return sample, score

    def strict_checker(self,x,y,d):
      return np.any(self.mapdata[x-d:x+d, y-d:y+d]==1)

    def sample_multi(self, num):
      '''map frame multi sampler'''
      sample_list = []
      score_list = []
      for i in range(num):
        sample, score = self.sample_free()
        sample_list.append(sample)
        score_list.append(score)
      samples = np.stack(sample_list, axis=0)
      scores = np.stack(score_list)
      return samples, scores

    def sample_in_world(self, num):
      '''world frame multi sampler'''
      samples, scores = self.sample_multi(num)
      samples = util.map_to_world(samples, self.map_info)
      samples[:,2] = scores
      return samples

    def output_samples(self, num):
      samples = self.sample_in_world(num)
      np.savetxt('../waypoints/1.txt', samples, fmt='%.4f', delimiter=',')
      print(samples)
      #np.savetxt('gen_waypts.txt', samples, delimiter=',')

    def get_map(self):
      '''
      get_map is a utility function which fetches a map from the map_server
      output:
          map_msg - a GetMap message returned by the mapserver
      '''
      srv_name = "/obs/static_map"
      #srv_name = "/static_map"
      #self.params.get_str("static_map", default="/static_map")
      #self.logger.debug("Waiting for map service")
      print "Waiting for map service"
      rospy.wait_for_service(srv_name)
      print "Map service started"
      #self.logger.debug("Map service started")

      map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
      return map_msg

if __name__ == '__main__':
  sampler = WaypointSampler()
  sampler.output_samples(1)
  wp_client = rospy.ServiceProxy("/final_planner/run_waypoints", ReadFile)
  wp_client("../waypoints/1.txt")


