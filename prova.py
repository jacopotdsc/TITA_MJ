import sys
sys.path.insert(0, '/home/ubuntu/Desktop/repo_rl//TITA_MJ/compiled')

import mpc
import wbc
import numpy as np
import pinocchio

x0 = np.zeros(10)
x0[2]=0.4
m=mpc.MPC()
m.solve(x0)
s=m.get_solution()
print('COM pos', s['com_pos'])


params = wbc.WholeBodyControllerParams.getDefaultParams()  # o creare i campi necessari
robot_model = pinocchio.buildSampleModelHumanoid()  # esempio, usa il tuo modello
initial_robot_state = wbc.RobotState()  # setta posizione e velocità iniziali
sample_time = 0.001  # esempio
armature = { "joint1": 0.1, "joint2": 0.1 }  # esempio

controller = wbc.WholeBodyController(params, robot_model, initial_robot_state, sample_time, armature)

# --- Passa la soluzione del MPC al WBC ---
des_config = wbc.DesiredConfiguration()  # inizializza e riempi con la soluzione MPC
des_config.com_position = s['com_pos']
des_config.joint_positions = s['q']  # se la soluzione contiene anche le joint

# Calcola inverse dynamics
tau = controller.compute_inverse_dynamics(initial_robot_state, des_config)
print('Torques from WBC:', tau)