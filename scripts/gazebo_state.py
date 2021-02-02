#!/usr/bin/env python

import rospy

import tf_conversions
import tf2_ros

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, Vector3, Twist, TransformStamped
from ford_msgs.msg import PedTrajVec, NNActions, PlannerMode, Clusters, PedTraj, Pose2DStamped

ped_num = 3

class GazeboLinkPose:  
  link_name = ''
  link_pose = PoseStamped()
  link_vel = Vector3()
  peds = PedTrajVec()

  def __init__(self, link_name):
    self.link_name = link_name

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name.replace("X1::", "pose_"), PoseStamped, queue_size = 10)
    self.vel_pub = rospy.Publisher("/gazebo/" + self.link_name.replace("X1::", "vel_"), Vector3, queue_size = 10)
    self.peds_pub = rospy.Publisher("/gazebo/pedtrajs", PedTrajVec, queue_size=2)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose.pose = data.pose[ind]
      self.link_pose.header.stamp = rospy.Time.now()
      self.link_pose.header.frame_id = "world"
      link_twist = Twist()
      link_twist = data.twist[ind]
      self.link_vel.x = link_twist.linear.x
      self.link_vel.y = link_twist.linear.y
      self.link_vel.z = 0

    #   br = tf2_ros.TransformBroadcaster()
    #   t = TransformStamped()
  
    #   t.header.stamp = self.link_pose.header.stamp
    #   t.header.frame_id = "world"
    #   t.child_frame_id = "X1/base_link"
    #   t.transform.translation = self.link_pose.pose.position
    #   t.transform.rotation = self.link_pose.pose.orientation
    #   br.sendTransform(t)

      peds_arr = []
      for i in range(1,ped_num+1):
          ind = data.name.index('ped' + str(i) + '::ped' + str(i) +'/base_link')
          pose = data.pose[ind]
          twist = data.twist[ind]
          
          ped = PedTraj()
          traj = Pose2DStamped()
          
          traj.header.frame_id = "world"
          traj.header.stamp = rospy.Time.now()
          traj.pose.x = pose.position.x
          traj.pose.y = pose.position.y
          traj.pose.theta = pose.orientation.z
          traj.velocity.x = twist.linear.x
          traj.velocity.y = twist.linear.y
          traj.velocity.z = 0

          ped.type = 0
          ped.ped_id = i
          ped.traj.append(traj)
          peds_arr.append(ped)
      self.peds.ped_traj_vec = peds_arr

    except ValueError:
      pass

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose('X1::X1/base_link')
    publish_rate = rospy.get_param('~publish_rate', 50)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      gp.vel_pub.publish(gp.link_vel)
      gp.peds_pub.publish(gp.peds)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
