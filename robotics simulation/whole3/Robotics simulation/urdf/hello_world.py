import pybullet as p
import time
import pybullet_data
import math
import crawl
import csv

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("whole3.urdf",cubeStartPos, cubeStartOrientation)
sphere = boxId
# stepId = p.loadURDF("step.urdf", [0.47, 0, 0.1], cubeStartOrientation)
print('number of joints: ', p.getNumJoints(boxId))
maxForce = 500
positions = []
jointstates = []
jointstates2 = []
for i in range (800):
	spherePos, orn = p.getBasePositionAndOrientation(sphere)
	if (i == 0 or i == 399):
		print(p.getBasePositionAndOrientation(boxId)[0])

	############# Stand ###############	
	if (i >= 300 and i <= 350):
		targetPos1 = - 30 / 180 * math.pi / 50 # -45
		targetPos2 = 45 / 180 * math.pi / 50 # 75
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

	a1 = - 45 / 180 * math.pi
	b1 = - (60 - 30) / 180 * math.pi / 2
	c1 = - math.pi / 2
	w1 = 0.05
	targetPos1 = a1 + b1 * math.sin(w1 * (i - 400) + c1)

	a2 = - 45 / 180 * math.pi
	b2 = - (60 - 30) / 180 * math.pi / 2
	c2 = - math.pi / 2
	w2 = 0.05
	targetPos2 = a2 + b2 * math.sin(w2 * (i - 400) + c2)
	if (i >= 400):
		p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
			targetPosition= targetPos1)
	# if (i >= 400):
	# 	p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -1/2 * math.sin((i-400)*0.05) - 1.2)
	# 	# p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
	# 	# 	targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +60/180*math.pi)
	# 	p.setJointMotorControl2(boxId, 10, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -1/2 * math.sin((i-400)*0.05) - 1.2 )
	# 	# p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
	# 	# 	targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +60/180*math.pi)
	# if (i >= 420):
	# 	p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +80/180*math.pi)
	# 	p.setJointMotorControl2(boxId, 11, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +80/180*math.pi)
	# if (i >= 450):
	# 	p.setJointMotorControl2(boxId, 4, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -1/2 * math.sin((i-450)*0.05) - 1.2)
	# 	p.setJointMotorControl2(boxId, 7, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -1/2 * math.sin((i-450)*0.05) - 1.2)
	# if (i >= 470):
	# 	p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((i-450)*0.05-math.pi/2) +80/180*math.pi)
	# 	p.setJointMotorControl2(boxId, 8, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((i-450)*0.05-math.pi/2) +80/180*math.pi)
	
	if (i >= 300):
		positions.append(p.getBasePositionAndOrientation(boxId)[0])
		# jointstates.append(p.getJointState(boxId, 1)[0])
		# jointstates2.append(p.getJointState(boxId, 2)[0])

### Try		
		# p.setJointMotorControl2(boxId, 6, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= -1.49 * math.sin((i-400)*0.01) + 0.41)
		# p.setJointMotorControl2(boxId, 9, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.49 * math.sin((i-400)*0.01) - 0.41)
		# p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.44 * math.sin((i-400)*0.01 - math.pi) + 1.44)
		# p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.44 * math.sin((i-400)*0.01 - math.pi) + 1.44)
		# p.setJointMotorControl2(boxId, 8, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.44 * math.sin((i-400)*0.01 - math.pi) + 1.44)
		# p.setJointMotorControl2(boxId, 11, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.44 * math.sin((i-400)*0.01 - math.pi) + 1.44)
		# p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 1.44 * math.sin((i-400)*0.01 - math.pi) + 1.44)
		# p.setJointMotorControl2(boxId, 4, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 3.24 / 2 * math.sin((i-400)*0.01 + math.pi/2) - 1.57)
		# p.setJointMotorControl2(boxId, 7, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 3.24 / 2 * math.sin((i-400)*0.01 + math.pi/2) - 1.57)
		# p.setJointMotorControl2(boxId, 10, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 3.24 / 2 * math.sin((i-400)*0.01 + math.pi/2) - 1.57)
	force = [-10, 0, 0]
	p.applyExternalForce(sphere, -1, force, spherePos, flags=p.WORLD_FRAME)
	p.stepSimulation()
	time.sleep(1./240.)

# with open('output4.csv', 'w', newline='') as csvfile:
# 	writer = csv.writer(csvfile)
# 	writer.writerow(['x', 'y', 'z'])
# 	for i in range(len(positions)):
# 		writer.writerow(positions[i])

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()