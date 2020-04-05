#!/usr/bin/env python
import rospy
import math
import threading
import sys, select, tty, termios
import way_points_yaw
import state_manager
import tf
import numpy as np


from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


last_time = None

class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_data(self):
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except:
            return '[CTRL-C]'
        return False

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

#reads the current state of the drone: Landed, hovering, etc.
copter_state_manager = state_manager.StateManager()
odom_current = Odometry()
bebop_pose = PoseStamped()

def odometry_callback(Odometry):
    global odom_current
    global odom_time
    odom_current = Odometry
    odom_time = rospy.get_time()

def pose_callback(Pose):
    global bebop_pose
    bebop_pose = PoseStamped

#reads input from keyboard to send commands to bebop
def key_reader():
    global is_running
    global copter_state_manager
    global is_keyreader_finished
    global speed_x, speed_y, speed_z
    global pub_takeoff, pub_landing
    global time_current

    message_empty = Empty()
    with NonBlockingConsole() as nbc:
        while is_running:
            c = nbc.get_data()
            time_current = rospy.get_time()
            # x1b is ESC, if key ESC is pressed publish a landing command
            if c == '\x1b':
                is_keyreader_finished = True
                pub_landing.publish(message_empty)
                break
            # if key 's' is pressed send a stop command to bebop
            elif c == 's':
                speed_x = 0.0
                speed_y = 0.0
                speed_z = 0.0
                speed_yaw = 0.0
                print "zero position"
            # if 'l' is pressed send a landing command to bebop
            elif c == 'l':
                copter_state_manager.set_state(state_manager.COPTER_STATE_LANDING, time_current)
                speed_x = 0.0
                speed_y = 0.0
                speed_z = 0.0
                speed_yaw = 0.0
                pub_landing.publish(message_empty)
                print "land"
            # if 'n' is pressed change the state to navigation mode
            elif c == 'n':
                current_state = copter_state_manager.get_state(time_current)
                if current_state == state_manager.COPTER_STATE_HOVERING:
                    print "navigate"
                    copter_state_manager.set_state(state_manager.COPTER_STATE_NAVIGATING, time_current)
                else:
                    print "Not in Hovering State yet"
            # if 't' is pressed send a command to bebop to take off
            elif c == 't':
                copter_state_manager.set_state(state_manager.COPTER_STATE_TAKING_OFF, time_current)
                pub_takeoff.publish(message_empty)
                print "take off"

def get_yaw_from_quaternion(quaternion):
    global Pose
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
    return yaw

counter = 0
points = way_points_yaw.points

#reads trajectory points from way_points_yaw.py and follows the target points, length is the length of error vector
def way_point(length,error_yaw):
    global counter
    global points
    num_points = len(points)
    #if we are closer to the target point less than 0.2 meters go to next point
    if length <= 0.2 and counter < num_points and abs(error_yaw) <= math.pi/9:
        print "going to next point"
        counter = counter + 1
        return [True, max(counter-1,0)]
    else:
        return [False, max(counter-1,0)]

def odomUpdated(time_current,max_delay):
    global odom_time
    delay = time_current - odom_time
    if delay <= max_delay:
        updated = True
    else:
        updated = False
    # print "delay: ", delay
    return updated

# kp : increase to increase speed of response
# ki : decrease to increase speed of response, affects stability
# kd : predicting error, Its output depends on rate of change of error with respect to time
now = {}
last_time = {'x':0,'y':0,'z':0,'yaw':0}
err_sum = {}
last_err = {}
time_change = {}
dErr = {}
output = {}

def pid (name,error,kp,ki,kd,limit):
    global time_current, last_time, last_err, err_sum
    # if "last_time" not in globals():

    now [name] = rospy.get_time()
    if last_time[name] == 0:
        last_time [name] = now[name]
        err_sum [name] = 0
        last_err [name] = 0
    time_change[name] = now[name] - last_time[name]
    err_sum[name] = (error * time_change[name]) + err_sum[name]
    if err_sum [name] > limit:
        err_sum [name] == limit
    elif err_sum [name] < (limit * -1):
        err_sum [name] == (limit * -1)
    if time_change [name] == 0:
        dErr[name] = 0
    else:
        dErr[name] = (error - last_err[name]) / time_change[name]
    output[name] = kp * error + ki * err_sum[name] + kd * dErr[name]

    last_err[name] = error
    last_time[name] = now[name]
    return output[name]

def bebop_position_controller_yaw_pid():
    global pub_landing, copter_state_manager
    global is_keyreader_finished
    global current_state, time_current
    global twist_current, odom_current
    global speed_x, speed_y, speed_z, graph

    graph_error = []
    graph_current = []

    graph_x = []
    graph_y = []
    graph_z = []

    graph_error_x = []
    graph_error_y = []
    graph_error_z = []

    graph_speed_x = []
    graph_speed_y = []
    graph_speed_z = []

    length = 0
    error_yaw = 0
    Pyaw = 0.3      
    rate = rospy.Rate(20)

    message_empty = Empty()
    set_pt = []

    while not rospy.is_shutdown() and not is_keyreader_finished:
        time_current = rospy.get_time()
        current_state = copter_state_manager.get_state(time_current)
	
        if current_state == state_manager.COPTER_STATE_NAVIGATING:
            _set_pt = way_point(length, error_yaw)
            current_index = _set_pt[1]
            if _set_pt[0]:
                set_pt = points[current_index]
            else:
                set_pt = points[current_index]
            current_x = odom_current.pose.pose.position.x
            current_y = odom_current.pose.pose.position.y
            current_z = odom_current.pose.pose.position.z
            bebop_yaw = get_yaw_from_quaternion(odom_current.pose.pose.orientation)

            error_x = set_pt[0] - current_x
            error_y = set_pt[1] - current_y
            error_z = set_pt[2] - current_z

	    error_yaw_calc = math.radians(set_pt[3]) - bebop_yaw

            if error_yaw_calc > math.pi:
		 error_yaw = error_yaw_calc - 2*math.pi
	    elif error_yaw_calc < -math.pi:
	    	 error_yaw = error_yaw_calc + 2*math.pi
	    else: 
		 error_yaw = error_yaw_calc 
  
            #calculates the length of error vector (target pose - current pose)
            length = math.sqrt(error_x **2 + error_y **2 + error_z **2)

            speed_x = pid('x',error_x,0.15,0.01,0.06,1)
            speed_y = pid('y',error_y,0.15,0.01,0.06,1)
            speed_z = pid('z',error_z,0.15,0.01,0.06,1)
            speed_yaw = pid('yaw',error_yaw,0.15,0.01,0.06,1)  

            graph_error.append([error_x,error_y,error_z,error_yaw])
            graph_current.append([current_x,current_y,current_z,bebop_yaw])

	    graph_x.append(current_x)
	    graph_y.append(current_y)
	    graph_z.append(current_z)

  	    graph_error_x.append(error_x)
   	    graph_error_y.append(error_y)
    	    graph_error_z.append(error_z)

  	    graph_speed_x.append(speed_x)
   	    graph_speed_y.append(speed_y)
    	    graph_speed_z.append(speed_z)


            #print "----------------------------------------"
            # print "target: ",   set_pt
            #print "current: ",  round(current_x,3), round(current_y,3), round(current_z,3), round(bebop_yaw,3)
            # print "error: ",    round(error_x,3),   round(error_y,3),   round(error_z,3),   round(error_yaw,3)
            #print "speed: ",    round(speed_x,3),   round(speed_y,3),   round(speed_z,3),   round(speed_yaw,3)
            # print "length: ",   length
            # print "----"
            # print "pid_x = ",   pid('x',error_x,0.2,0.20.2)
            # print "pid_y = ",   pid('y',error_y,0.2,0.2,0.2)
            # print "pid_z = ",   pid('z',error_z,0.2,0.2,0.2)

	    #save poses error and current to txt file
#	    with open('poses_error.txt', 'w') as file:
#		file.write(str(graph_error))
	

	    file = open('poses_error.txt', 'w')
	    for i in graph_error:
		file.write(str(i) +  "\n")


	    with open('poses_current.txt', 'w') as file:
		file.write(str(graph_current))


	    with open('graph_x_list.txt', 'w') as file:
		file.write(str(graph_x))

	    with open('graph_y_list.txt', 'w') as file:
		file.write(str(graph_y))

	    with open('graph_z_list.txt', 'w') as file:
		file.write(str(graph_z))


	    with open('graph_error_x_list.txt', 'w') as file:
		file.write(str(graph_error_x))

	    with open('graph_error_y_list.txt', 'w') as file:
		file.write(str(graph_error_y))

	    with open('graph_error_z_list.txt', 'w') as file:
		file.write(str(graph_error_z))


	    with open('graph_speed_x_list.txt', 'w') as file:
		file.write(str(graph_speed_x))

	    with open('graph_speed_y_list.txt', 'w') as file:
		file.write(str(graph_speed_y))

	    with open('graph_speed_z_list.txt', 'w') as file:
		file.write(str(graph_speed_z))


            #if last update was within 1 second continue flying, otherwise land
            if odomUpdated(time_current,1) == True:
                twist_current = Twist(Vector3(speed_x,speed_y,speed_z),Vector3(0.0, 0.0, speed_yaw))
                # twist_current = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))
            else:
                twist_current = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))
                pub_landing.publish(message_empty)
                print "Cannot Update Odometry >> Landing"

            pub.publish(twist_current)
        rate.sleep()

if __name__ == '__main__':
    try:
        # initialize a node called bebop_position_controller_yaw_pid
        rospy.init_node('bebop_position_controller_yaw_pid', anonymous=True)
        # subscribes to bebop/odometry
        rospy.Subscriber("bebop/odom", Odometry, odometry_callback)
        # subscribe to a topic "setpoint" using geometry_msgs/poseStamped
        rospy.Subscriber("setpoint", PoseStamped, pose_callback)
        # publish cmd_vel
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        twist_current = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        # additional options
        pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        pub_landing = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        is_running = True
        thread_reader = threading.Thread(target=key_reader)
        thread_reader.start()
        is_keyreader_finished = False
        bebop_position_controller_yaw_pid()
        is_running = False
    except rospy.ROSInterruptException:
        pass
