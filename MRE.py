import URBasic
import time
import keyboard
import tkinter as tk

host = '169.254.56.120'   # E.g. a Universal Robot offline simulator, please adjust to match your IP
acc = 1.2
vel = 0.1

def move_robot(value):
    global target_tcp_pose
    target_tcp_pose[0] = float(value)
    robot.set_realtime_pose(pose=target_tcp_pose)

def increase_slider(event):
    position_slider.set(position_slider.get() + 0.001)

def decrease_slider(event):
    position_slider.set(position_slider.get() - 0.001)

if __name__ == '__main__':
    robot_model = URBasic.model.RobotModel()
    robot = URBasic.scriptExt.UrScriptExt(host=host, robotModel=robot_model)
    robot.robotConnector.DashboardClient.ur_close_popup()
    robot.reset_error()
    print("robot initialised")
    time.sleep(1)

    # Get and print the initial TCP pose
    inital_tcp_pose = robot.get_actual_tcp_pose()

    target_tcp_pose = inital_tcp_pose

    robot.init_realtime_control()
    time.sleep(1)

    try:
        print("starting loop")
        
        # Create Tkinter window
        root = tk.Tk()
        root.title("Robot Control")

        # Create and place slider
        global position_slider
        position_slider = tk.Scale(root, from_=-.1, to=.1, resolution=0.001, orient=tk.HORIZONTAL, label="Position", length=600, command=move_robot)
        position_slider.set(target_tcp_pose[0])
        position_slider.pack(pady=20)

        # Bind arrow keys to slider control functions
        root.bind('<Up>', increase_slider)
        root.bind('<Down>', decrease_slider)

        # Start Tkinter event loop
        root.mainloop()

        robot.close()

    except KeyboardInterrupt:
        print("closing robot connection")
        # Remember to always close the robot connection, otherwise it is not possible to reconnect
        robot.close()

    except:
        robot.close()