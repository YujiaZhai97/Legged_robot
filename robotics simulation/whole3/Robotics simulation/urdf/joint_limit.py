-3.24 * math.sin((i-400)*0.01) + 0.05
<limit effort="1.5" lower="0.05" upper="-3.19" velocity="5.8"/>
<limit effort="1.5" lower="0" upper="2.88" velocity="5.8"/>
<limit effort="1.5" lower="1.08" upper="-1.9" velocity="5.8"/>
p.setJointMotorControl2(boxId, 1, controlMode=p.POSITION_CONTROL, 
			targetPosition= 3.24 / 2 * math.sin((i-400)*0.01 + math.pi/2) - 1.57)

<mass
        value="0.0520137835226923" />