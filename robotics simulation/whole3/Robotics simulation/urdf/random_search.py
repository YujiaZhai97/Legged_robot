import pybullet as p
import time
import pybullet_data
import math
import csv
import random
import numpy as np
import matplotlib.pyplot as plt

# from gaits import stand
def stand():
	targetPos1 = - 30 / 180 * math.pi
	targetPos2 = 45 / 180 * math.pi
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=1,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)

	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=2,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)

	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=4,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=5,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=7,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=8,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=10,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos1,
                      force = maxForce)
	p.setJointMotorControl2(bodyUniqueId=boxId,
	                  jointIndex=11,
                      controlMode=p.POSITION_CONTROL,
                      targetPosition = targetPos2,
                      force = maxForce)

physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0, 0, 0.2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("whole3.urdf",cubeStartPos, cubeStartOrientation)
maxForce = 500
max_distance = -np.inf
max_distance_list = []
dist_set = []  # record the distance of every iteration # line 141
ite_set = []


iterations = 20
best_params = {}
# best_one = []



### params for shoulder joints ###
a_s = - 45 / 180 * math.pi
b_s = - (60 - 30) / 180 * math.pi / 2
w_s = 0.05
c_s = [-2.804294668930453, -0.5946210730977946, -1.5707963267948966, -5.390875268393517]
joint_indices_s = [1, 4, 7, 10]

### params for ankle joints ###
a_a = 45 / 180 * math.pi
b_a = (60 - 30) / 180 * math.pi / 2
w_a = 0.05
c_a = [0, 0, 0, 0]

joint_indices_a = [2, 5, 8, 11]
for iteration in range(iterations):
	
	pos_1 = []
	vel_1 = []
	torq_1 = []
	t = []
	fall = False
	boxId = p.loadURDF("whole3.urdf",cubeStartPos, cubeStartOrientation)
	sphere = boxId
	spherePos, orn = p.getBasePositionAndOrientation(sphere)
	print('Iteration: ', iteration)
	### params for shoulder joints ###
	# a_s += np.random.uniform(-45 / 180 * math.pi, 45 / 180 * math.pi)
	a_s = np.random.normal(- 45 / 180 * math.pi, 0.2)
	b_s += random.uniform(- 30 / 180 * math.pi / 2, 15 / 180 * math.pi / 2)
	k = random.randint(0, 3)
	c_s[k] += random.uniform(- math.pi, math.pi)
	print('A_s:', a_s)
	print('B_s:', b_s)
	print('C_s:', c_s)

	### params for ankle joints ###
	# a_a += np.random.uniform(-45 / 180 * math.pi, 45 / 180 * math.pi)
	a_a = np.random.normal(45 / 180 * math.pi, 0.1)
	b_a += random.uniform(- 30 / 180 * math.pi / 2, 50 / 180 * math.pi / 2)
	c_a[0] += random.uniform(- math.pi, math.pi)
	c_a[1] += random.uniform(- math.pi, math.pi)
	c_a[2] += random.uniform(- math.pi, math.pi)
	c_a[3] += random.uniform(- math.pi, math.pi)
	print('A_a:', a_a)
	print('B_a:', b_a)
	print('C_a:', c_a)

	start_x = p.getBasePositionAndOrientation(boxId)[0][0]
	# print('Start at: ', start_x)

	### Get the robot move for a certain period of time
	for i in range (500):
		
		############# Stand ###############	
		stand()
		z = p.getBasePositionAndOrientation(boxId)[0][2]
		# print('Z', z)
		if (z > 0.4 or z < 0.15):
			fall = True
			print('Fall or Fly!!!!!!!')
			# ite_set.append(iteration)
			# if(i==0):
			# 	dist_set.append(0)
			# else:
			# 	dist_set.append(dist_set[iteration-1]) # if some iteration is not successful, we use the last distance
			break;
		if (i >= 150):
			for j, joint_index in enumerate(joint_indices_s):
				targetPos = a_s + b_s * math.sin(w_s * (i - 150) + c_s[j])
				p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
						targetPosition= targetPos)
			for j, joint_index in enumerate(joint_indices_a):
				targetPos = a_a + b_a * math.sin(w_a * (i - 150) + c_a[j]) 
				p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
						targetPosition= targetPos)
		
		force = [5, 0, 0]
		p.applyExternalForce(sphere, -1, force, spherePos, flags=p.WORLD_FRAME)
		p.stepSimulation()

		pos, vel, force, torq  = p.getJointState(boxId, 1)
		pos_1.append(pos)
		vel_1.append(vel)
		torq_1.append(torq)
		t.append(i)

		# print("!!!here we get pos", pos)
		time.sleep(1./240.)
	end_x = p.getBasePositionAndOrientation(boxId)[0][0]
	# print('End at: ', end_x)
	distance = abs(end_x - start_x)
	ite_set.append(iteration) # record every random search
	dist_set.append(distance)
	# print("all distance we have", dist_set)
	if fall:
		distance = 0
	print('Distance: ', distance)
	if (distance > max_distance):
		max_distance = distance
		# best_params = c_s	
		best_params['a_s'] = a_s
		best_params['b_s'] = b_s
		best_params['c_s_1'] = c_s[0]
		best_params['c_s_2'] = c_s[1]
		best_params['c_s_3'] = c_s[2]
		best_params['c_s_4'] = c_s[3]
		best_params['a_a'] = a_a
		best_params['b_a'] = b_a
		best_params['c_a_1'] = c_a[0]
		best_params['c_a_2'] = c_a[1]
		best_params['c_a_3'] = c_a[2]
		best_params['c_a_4'] = c_a[3]
		best_one = [pos_1, vel_1, torq_1, t]
	print('Max distance: ', max_distance)
	max_distance_list.append(max(dist_set))
	
	p.removeBody(boxId)

print(dist_set)
print(max_distance_list)	
plt.plot(best_one[3],best_one[0], best_one[3], best_one[1], best_one[3],  best_one[2])
plt.legend(('angle position', 'angle speed', 'motor torque' ))
plt.title('Position, speed and motor torque of best result %.5f' %max_distance_list[-1])
plt.show()



plt.plot(ite_set, dist_set, ite_set, max_distance_list)
plt.ylabel('distance in a period of time')
plt.xlabel('iteration')
plt.show()
print('Best params: ', best_params)
print('Max distance: ', max_distance)
p.disconnect()
print('-------GUI-------')
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
a_s = best_params['a_s']
b_s = best_params['b_s']
w_s = w_s
c_s = [best_params['c_s_1'], best_params['c_s_2'], best_params['c_s_3'], best_params['c_s_4']]
a_a = best_params['a_a']
b_a = best_params['b_a']
w_a = w_a
c_a = [best_params['c_a_1'], best_params['c_a_2'], best_params['c_a_3'], best_params['c_a_4']]
boxId = p.loadURDF("whole3.urdf",cubeStartPos, cubeStartOrientation)
start_x = p.getBasePositionAndOrientation(boxId)[0][0]
# print('Start at: ', start_x)

### Get the robot move for a certain period of time
for i in range (500):
	############# Stand ###############	
	stand()

	if (i >= 150):
		for j, joint_index in enumerate(joint_indices_s):
			targetPos = a_s + b_s * math.sin(w_s * (i - 150) + c_s[j])
			p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
					targetPosition= targetPos)
		for j, joint_index in enumerate(joint_indices_a):
			targetPos = a_a + b_a * math.sin(w_a * (i - 150) + c_a[j])
			p.setJointMotorControl2(boxId, joint_index, controlMode=p.POSITION_CONTROL, 
					targetPosition= targetPos)
					
	p.stepSimulation()
	time.sleep(1./240.)
p.disconnect()
with open('shoulder_ankle_a_b_c.csv', 'w', newline='') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(['max_distance_list'] + max_distance_list)
	writer.writerow(['max_distance', max_distance])

	for key, value in best_params.items():
		writer.writerow([key, value])


