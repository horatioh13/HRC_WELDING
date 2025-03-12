import URBasic
import time
import keyboard
import threading
import pygame

host = '169.254.56.120'   # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 1.2
vel = 0.1

def update_pose():
    global target_tcp_pose
    step = 0.01  # Define the step size for each key press
    while True:
        if keyboard.is_pressed('w'):
            target_tcp_pose[0] += step
        if keyboard.is_pressed('s'):
            target_tcp_pose[0] -= step
        if keyboard.is_pressed('a'):
            target_tcp_pose[1] -= step
        if keyboard.is_pressed('d'):
            target_tcp_pose[1] += step
        if keyboard.is_pressed('q'):
            target_tcp_pose[2] += step
        if keyboard.is_pressed('e'):
            target_tcp_pose[2] -= step
        time.sleep(0.00001)  # Add a small delay to prevent high CPU usage


if __name__ == '__main__':
    #pygame.init()
    #pygame.joystick.init()
    #joystick = pygame.joystick.Joystick(0)
    #joystick.init()


    robot_model = URBasic.model.RobotModel()
    robot = URBasic.scriptExt.UrScriptExt(host=host, robotModel=robot_model)
    robot.robotConnector.DashboardClient.ur_close_popup()
    robot.reset_error()
    print("robot initialised")
    time.sleep(1)

    # Get and print the initial TCP pose
    inital_tcp_pose = robot.get_actual_tcp_pose()

    # Define a small displacement
    #displacement = [0, 0, 0, 0, 0, 0]  # Move 5 cm along the X-axis

    # Calculate the new pose
    #target_tcp_pose = [inital_tcp_pose[i] + displacement[i] for i in range(6)]
    #robot.movej(pose=new_tcp_pose, a=acc, v=vel)
    target_tcp_pose = inital_tcp_pose

    robot.init_realtime_control()
    time.sleep(1)

    # Start a new thread to monitor keyboard input
    #threading.Thread(target=update_pose, daemon=True).start()

    try:
        print("starting loop")
        while True:
            step = 0.001  # Define the step size for each key press
            if keyboard.is_pressed('w'):
                target_tcp_pose[0] += step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue
            elif keyboard.is_pressed('s'):
                target_tcp_pose[0] -= step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue
            elif keyboard.is_pressed('a'):
                target_tcp_pose[1] -= step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue
            elif keyboard.is_pressed('d'):
                target_tcp_pose[1] += step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue
            elif keyboard.is_pressed('q'):
                target_tcp_pose[2] += step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue
            elif keyboard.is_pressed('e'):
                target_tcp_pose[2] -= step
                robot.set_realtime_pose(pose=target_tcp_pose)
                time.sleep(0.00001)  # Add a small delay to prevent high CPU usage
                continue



    except KeyboardInterrupt:
        print("closing robot connection")
        # Remember to always close the robot connection, otherwise it is not possible to reconnect
        robot.close()

    except:
        robot.close()