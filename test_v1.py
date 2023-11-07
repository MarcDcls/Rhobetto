import time
import numpy as np
import placo
from placo_utils.visualization import robot_viz

robot = placo.RobotWrapper("rhobetto_v1")
solver = robot.make_solver()

robot.set_T_world_frame("left", np.eye(4))
robot.update_kinematics()

gear_tasks = solver.add_gear_task()
for dof in robot.actuated_joint_names():
    if "1" in dof:
        gear_tasks.set_gear(dof[:-1] + "2", dof, 1)
gear_tasks.configure("gears", "hard", 0)

for frame in robot.frame_names():
    if "1_frame" in frame:
        task = solver.add_relative_orientation_task(frame, frame[:-7] + "2_frame", np.eye(3))
        task.mask.set_axises("y", "local")
        task.configure(frame[:-7] + "relative_orientation", "hard", 0)

robot.set_joint_limits("left_knee_3", -np.pi, 1e-3)
robot.set_joint_limits("right_knee_3", -1e-3, np.pi)

regularization_task = solver.add_regularization_task(1e-3)

# joints_task = solver.add_joints_task()

com_init = robot.com_world()
com_task = solver.add_com_task(com_init)
# com_task.mask.set_axises("z")

T_world_left = placo.flatten_on_floor(robot.get_T_world_frame("left"))
left_foot_task = solver.add_frame_task("left", T_world_left)
left_foot_task.configure("left_foot", "soft", 1e3, 1e3)

T_world_right = placo.flatten_on_floor(robot.get_T_world_frame("right"))
right_foot_task = solver.add_frame_task("right", T_world_right)
right_foot_task.configure("right_foot", "soft", 1e3, 1e3)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))

viz = robot_viz(robot)
viz.display(robot.state.q)

t0 = time.time()
while(1):
    t = time.time() - t0
    # joints_task.set_joint("left_hip_roll_1", 0.5 * np.sin(t))

    com_task.target_world = com_init + np.array([0, 0, 0.05 * (np.sin(t) - 1)])
    robot.update_kinematics()
    solver.solve(True)
    # solver.dump_status()

    viz.display(robot.state.q)
