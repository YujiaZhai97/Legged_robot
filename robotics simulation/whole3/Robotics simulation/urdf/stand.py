import pybullet as p
import math

def stand():
	# targetPositions = [targetPos1, targetPos2, targetPos1, targetPos2, targetPos1, targetPos2, targetPos1, targetPos2]
	# maxForces = [maxForce, maxForce, maxForce, maxForce, maxForce, maxForce, maxForce, maxForce]
	# p.setJointMotorControlArray(bodyUniqueId=boxId,
 #                  jointIndices=[1, 2, 4, 5, 7, 8, 10, 11],
 #                  controlMode=p.POSITION_CONTROL,
 #                  targetPositions = targetPositions,
 #                  forces = maxForces)
	targetPos1 = - 45 / 180 * math.pi / 50
	targetPos2 = 75 / 180 * math.pi / 50
	if (i >= 300 and i <= 350):
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=1,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos1 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=2,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos2 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=4,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos1 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=5,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos2 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=7,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos1 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=8,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos2 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=10,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos1 * (i - 300),
	                      force = maxForce)
		p.setJointMotorControl2(bodyUniqueId=boxId,
		                  jointIndex=11,
	                      controlMode=p.POSITION_CONTROL,
	                      targetPosition = targetPos2 * (i - 300),
	                      force = maxForce)