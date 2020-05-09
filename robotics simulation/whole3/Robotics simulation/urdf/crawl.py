def crawl():
	# Walk
	if (i >= 400):
		p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
			targetPosition= -1/2 * math.sin((i-400)*0.05) - 1.2)
		# p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +60/180*math.pi)
		p.setJointMotorControl2(boxId, 4, controlMode=p.POSITION_CONTROL, 
			targetPosition= -1/2 * math.sin((i-400)*0.05) - 1.2 )
		# p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
		# 	targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +60/180*math.pi)
	if (i >= 420):
		p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
			targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +80/180*math.pi
								+ 15 / 180 * math.pi * math.sin((i-400)*0.02-math.pi/2))
		p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
			targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +80/180*math.pi
								+ 15 / 180 * math.pi * math.sin((i-400)*0.02-math.pi/2))
	# if (i>=463 and i<926):
	# 	p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -0.52/2 * math.sin((i-400)*0.05) - 0.78)
	# 	p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((463-400)*0.1-math.pi/2) +60/180*math.pi)
	# if (i >= 962):
	# 	p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -0.52/2 * math.sin((i-400)*0.05) - 0.78)
	# 	p.setJointMotorControl2(boxId, 2, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 30 / 180 * math.pi * math.sin((i-400)*0.05-math.pi/2) +60/180*math.pi)
	# 	p.setJointMotorControl2(boxId, 5, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 15 / 180 * math.pi * math.sin((i-400)*0.1-math.pi) +75/180*math.pi)
	# if (i >= 450):
	# 	p.setJointMotorControl2(boxId, 10, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= -0.52/2 * math.sin((i-450)*0.05) - 0.78)
	# 	p.setJointMotorControl2(boxId, 11, controlMode=p.POSITION_CONTROL, 
	# 		targetPosition= 15 / 180 * math.pi * math.sin((i-450)*0.1-math.pi) +75/180*math.pi)
