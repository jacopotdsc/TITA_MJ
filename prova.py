import sys
sys.path.insert(0, '/home/ubuntu/Desktop/repo_rl//TITA_MJ/compiled')

import mpc
import wbc
import numpy as np
import pinocchio as pin
import traceback

x0 = np.zeros(10)
x0[2]=0.4
m=mpc.MPC()
m.solve(x0)
s=m.get_solution()
print('COM pos', s['com_pos'])


params = wbc.WholeBodyControllerParams.getDefaultParams()  # o creare i campi necessari

xml_path = "/home/ubuntu/Desktop/repo_rl/TITA_MJ/tita_description/tita.urdf"  
mesh_dir = "./tita_description"

try:
    root_joint = pin.JointModelFreeFlyer() 
    full_robot_model = pin.buildModelsFromUrdf(
        xml_path, 
        mesh_dir,
        root_joint
    )[0]
    print("Robot model loaded successfully.")
except Exception as e:
    print("Error loading robot model:", e)
    traceback.print_exc()
    sys.exit(1)

try:

    joint_to_lock_names = []
    joint_ids_to_lock = [full_robot_model.getJointId(name) 
                        for name in joint_to_lock_names 
                        if full_robot_model.existJointName(name)]
    q0 = pin.neutral(full_robot_model)
    robot_model = pin.buildReducedModel(full_robot_model, joint_ids_to_lock, q0)
except Exception as e:
    print("Error building reduced robot model:", e)
    traceback.print_exc()
    sys.exit(1)

try:
    initial_robot_state = wbc.RobotState()  # setta posizione e velocità iniziali
    sample_time = 0.001  
    armature = { "joint1": 0.1, "joint2": 0.1 }  # esempio
except Exception as e:
    print("Error initializing robot state or parameters:", e)
    traceback.print_exc()
    sys.exit(1)


print('Initialize WBC...')
try:
    controller = wbc.WholeBodyController(params, xml_path, mesh_dir, initial_robot_state, sample_time, armature)
    print("WBC initialized successfully.")
except Exception as e:
    print("Error initializing WBC:", e)
    traceback.print_exc()
    sys.exit(1)
    
# --- Passa la soluzione del MPC al WBC ---
print("Setting desired configuration from MPC solution...")
try:
    des_config = wbc.DesiredConfiguration()  # inizializza e riempi con la soluzione MPC
    print("DesiredConfiguration created successfully.")
except Exception as e:
    print("Error creating DesiredConfiguration:", e)
    traceback.print_exc()
    sys.exit(1)


    
try:
    des_config.position = s['com_pos']
    des_config.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
    des_config.qjnt = np.zeros(8)
    print("DesiredConfiguration fields set successfully.")
except Exception as e:
    print("Error setting DesiredConfiguration fields:", e)
    traceback.print_exc()
    sys.exit(1)

print("Computing inverse dynamics...")
print("position:", des_config.position)
print("orientation:", des_config.orientation)
print("qjnt:", des_config.qjnt)
print("linear_velocity:", des_config.linear_velocity)

try:
    tau = controller.compute_inverse_dynamics(initial_robot_state, des_config)
    print('Torques from WBC:', tau)
except Exception as e:
    print("Error computing inverse dynamics:", e)
    traceback.print_exc()
    sys.exit(1)

print("WBC computation completed.")