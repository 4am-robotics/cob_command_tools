#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from kinematics_msgs.srv import *
from tf.transformations import *

sss = simple_script_server()

def getPoseStamped(pos,quat):
    ps = PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = pos  
    ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quat
    return ps
 
class TestScript:
		
	def __init__(self):
		sss.init("arm")
		self.fks = rospy.ServiceProxy('/cob_arm_kinematics/get_fk', GetPositionFK)
		self.iks = rospy.ServiceProxy('/cob_arm_kinematics/get_ik', GetPositionIK)

	def callFK(self, joint_values):
		req = GetPositionFKRequest()
		req.header.stamp = rospy.Time.now()
		req.header.frame_id="base_footprint"
		req.fk_link_names = ['arm_7_link']
		req.robot_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
		req.robot_state.joint_state.position = joint_values
		
		res = self.fks(req)
		#print res
		return res.pose_stamped if res.error_code.val == res.error_code.SUCCESS else None
		
	def callIK(self, pose_stamped):
		req = GetPositionIKRequest()
		req.timeout = rospy.Duration(5)
		req.ik_request.ik_link_name = "arm_7_link"
		req.ik_request.ik_seed_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
		req.ik_request.ik_seed_state.joint_state.position = [0]*7
		req.ik_request.pose_stamped = pose_stamped
	    
		res = self.iks(req)
		return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code
		
	def Run(self): 
            pos =  [-0.494, -0.772, 1.414]
	    quat =  [0.291, -0.539, 0.593, 0.522]
            #fk = self.callFK([0]*7)
	    #print fk
	    #if fk:		
	        #ik, error_code = sss.calculate_ik(['base_footprint',pos, euler_from_quaternion(quat)])
	        #print error_code,ik
                #if ik:
                #    pass
            pose_target = PoseStamped()
            pose_origin = PoseStamped()
            pose_target.header.stamp = pose_origin.header.stamp = rospy.Time.now()
            pose_origin.header.frame_id = "arm_7_link"
            pose_origin.pose.orientation.w = 1.0
            pose_target.pose.orientation.w = 1.0
#            pose_target.header.frame_id = "base_laser_rear_link"
#            pose_target.pose.position.x, pose_target.pose.position.y, pose_target.pose.position.z = [0.2,0.0,0.4]
#            pose_target.pose.orientation.x,pose_target.pose.orientation.y, pose_target.pose.orientation.z, pose_target.pose.orientation.w = quaternion_from_euler(-1.57,0,-1.57)

            pose_target.header.frame_id = "arm_7_link"
            pose_target.pose.position.x, pose_target.pose.position.y, pose_target.pose.position.z = [-0.1,0.0,0]
            sss.move_pose_goal_planned('arm',[pose_target,pose_origin])
            exit()
		
if __name__ == "__main__":
        rospy.init_node("TestIK")
	SCRIPT = TestScript()
	SCRIPT.Run()
	rospy.spin()

