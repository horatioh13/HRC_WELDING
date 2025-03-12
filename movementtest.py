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
    print("robot initialised")
    time.sleep(1)

    # Get and print the initial TCP pose
    inital_tcp_pose = robot.get_actual_tcp_pose()

    # Define a small displacement
    displacement = [-0.2, 0, 0, 0, 0, 0]  # Move 5 cm along the X-axis

    # Calculate the new pose
    target_tcp_pose = [inital_tcp_pose[i] + displacement[i] for i in range(6)]
    #robot.movej(pose=new_tcp_pose, a=acc, v=vel)

    robot.init_realtime_control()
    time.sleep(1)

    try:
        print("starting loop")
        while True:
            robot.set_realtime_pose(pose=target_tcp_pose)

    except KeyboardInterrupt:
        print("closing robot connection")
        # Remember to always close the robot connection, otherwise it is not possible to reconnect
        robot.close()

    except:
        robot.close()