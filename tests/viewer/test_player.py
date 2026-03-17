from compas_fab.robots.robot_library import RobotCellLibrary
from compas_fab.viewer import TrajectoryPlayer  # <-- Look how clean this import is!
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint

# 1. Load your cell
robot_cell, cell_state = RobotCellLibrary.ur10e()

# 2. Make dummy trajectory
trajectory_points = []
for i in range(101):
    angle = (i / 100.0) * -1.57
    trajectory_points.append(JointTrajectoryPoint(
        joint_values=[angle]*6, joint_types=[0]*6
    ))
trajectory = JointTrajectory(trajectory_points=trajectory_points, joint_names=robot_cell.robot_model.get_configurable_joint_names())

# 3. Launch it!
player = TrajectoryPlayer(robot_cell, trajectory, cell_state)
player.show()