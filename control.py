import time
import numpy as np
import placo
from placo_utils.visualization import robot_viz, robot_frame_viz

robot = placo.RobotWrapper("rhobetto")
solver = robot.make_solver()

robot.set_T_world_frame("left", np.eye(4))
robot.update_kinematics()

gear_tasks = solver.add_gear_task()
for dof in robot.actuated_joint_names():
    if "1" in dof:
        gear_tasks.set_gear(dof[:-1] + "2", dof, 1)
gear_tasks.configure("gears", "hard", 0)

left_closing = solver.add_relative_position_task("closing_left_1", "closing_left_2", np.zeros(3))
left_closing.mask.set_axises("xy", "local")
left_closing.configure("left_closing", "hard", 0)

right_closing = solver.add_relative_position_task("closing_right_1", "closing_right_2", np.zeros(3))
right_closing.mask.set_axises("xy", "local")
right_closing.configure("right_closing", "hard", 0)

regularization_task = solver.add_regularization_task(1e-3)

# joints_task = solver.add_joints_task()

com_init = robot.com_world()
com_task = solver.add_com_task(com_init)
# com_task.mask.set_axises("z")

T_world_left = placo.flatten_on_floor(robot.get_T_world_frame("left"))
left_foot_task = solver.add_frame_task("left", T_world_left)
left_foot_task.configure("left_foot", "soft", 1e3, 1e3)

T_world_right = placo.flatten_on_floor(robot.get_T_world_frame("right"))
T_world_right[1, 3] -= 0.1
right_foot_task = solver.add_frame_task("right", T_world_right)
right_foot_task.configure("right_foot", "soft", 1e3, 1e3)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))

viz = robot_viz(robot)
viz.display(robot.state.q)

t0 = time.time()
while(1):
    t = time.time() - t0

    # com_task.target_world = com_init + np.array([0, 0, 0.075 * (np.sin(t) - 1)])
    # com_task.target_world = com_init + np.array([0, 0.04 * np.sin(t), -0.1])
    com_task.target_world = com_init + np.array([0, 0.05 * np.sin(t) - 0.05, -0.1])

    robot.update_kinematics()
    solver.solve(True)

    viz.display(robot.state.q)