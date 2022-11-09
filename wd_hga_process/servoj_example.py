from rtde_control import RTDEControlInterface as RTDEControl
import time

rtde_c = RTDEControl("192.168.12.100")

# Parameters
velocity = 0.03
acceleration = 0.01
dt = 1.0  # 2ms
lookahead_time = 1.0
gain = 1
joint_q = [ [2.9184842109680176, -1.2875714165023346, 1.0661171118365687, -1.3477059614709397, -1.5630133787738245, 2.867312431335449],
            [1.186805009841919, -1.8805929623045863, 1.7243402639972132, -1.3963409227183838, -1.5687468687640589, 2.7631101608276367]
          ]

# Move to initial joint position with a regular moveJ
t1 = time.time()
print('joint_q[0]')
rtde_c.moveJ(joint_q[0], 0.05, 0.25, False)
t2 = time.time()
print(t2-t1)
rtde_c.stopJ(0.5)
print('joint_q[0]')
rtde_c.moveJ(joint_q[1], 0.05, 0.25, True)
t3 = time.time()
print(t3-t2)
rtde_c.stopJ(0.5)
rtde_c.servoStop()
rtde_c.stopScript()

