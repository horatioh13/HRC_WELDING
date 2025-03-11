import URBasic
import time

host = '169.254.56.120'   # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 1.2
vel = 0.1

if __name__ == '__main__':
    robot_model = URBasic.model.RobotModel()
    robot = URBasic.scriptExt.UrScriptExt(host=host, robotModel=robot_model)
    robot.robotConnector.DashboardClient.ur_close_popup()
    robot.reset_error()

    # Get and print the initial TCP pose
    initial_tcp_pose = robot.get_actual_tcp_pose()
    print(f"Initial TCP Pose: {initial_tcp_pose}")

    # Define a small displacement
    displacement = [0.05, 0, 0, 0, 0, 0]  # Move 5 cm along the X-axis

    # Calculate the new pose
    new_tcp_pose = [initial_tcp_pose[i] + displacement[i] for i in range(6)]

    # Move to the new pose
    print('Moving to new pose...')
    robot.movel(pose=new_tcp_pose, a=acc, v=vel)

    # Get and print the new TCP pose
    final_tcp_pose = robot.get_actual_tcp_pose()
    print(f"Final TCP Pose: {final_tcp_pose}")

    # Close the connection to the robot
    robot.close()