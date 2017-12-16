import openravepy 
import Kinematics as kin
import MotionFunctions as mf
import numpy as np
import sys
import socket
import this

# handles the data transfer between openrave (server) and the GUI (client)
def dataTransfer():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#      
#     try:
#         s.bind(('localhost', 54321))
#         while True:
#             data, addr = s.recvfrom(2048)   
#             send = handleData(data)
#             # send new informations to the GUI for updating purposes
#             s.sendto(send, addr)
#     except:
#         while True:
#             data, addr = s.recvfrom(2048)   
#             send = handleData(data)
#             # send new informations to the GUI for updating purposes
#             s.sendto(send, addr)
#     finally:
#         s.close()
        
    UDP_IP = "localhost"
    UDP_PORT = 54321
      
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((UDP_IP, UDP_PORT))
        while True:
            data, addr = sock.recvfrom(2048)
            send = handleData(data) 
            sock.sendto(send, addr)
            print data
    finally:
        sock.close()
    


    

# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        # prefix for parsing
        prefix = "VAL#"
        # get Axis values
        axis_arr = robot.GetDOFValues()
        # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
        # adding dummy values for orientation and position (you need to compute the values)
        jsv = kin.vectorspace(axis_arr)
        tcp = jsv.base2tcp()
        cart_values = tcp.cart()
        return prefix+axis_values+cart_values

        #cart_values = "0;0;0;0;0;0"
        #return prefix+axis_values+cart_values
    
    # check if the robot should be moved 
    elif data_arr[0] == 'MOV':
        # get values
        values = data_arr[1].split(';')
        # convert from string to float and save in numpy array
        target = np.array([float(values[0]), float(values[1]), float(values[2]), float(values[3]), float(values[4]), float(values[5])])
                
        # get the motion type
        motion_type = data_arr[2]
        
        # get trajectory
        trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
        # move robot
        mf.Move(robot, trajectory)
        
        # send new information about the robot's axis values, position and orientation to the GUI for updating purpose
        # prefix for parsing
        prefix = "VAL#"
    # get Axis values
        axis_arr = robot.GetDOFValues()
    # convert to string
        axis_values = str(axis_arr[0])+";"+str(axis_arr[1])+";"+str(axis_arr[2])+";"+str(axis_arr[3])+";"+str(axis_arr[4])+";"+str(axis_arr[5])+'#'
    # adding dummy values for orientation and position (you need to compute the values)
        jsv = kin.vectorspace(axis_arr)
        tcp = jsv.base2tcp()
        cart_values = tcp.cart()
        return prefix+axis_values+cart_values

        
        #cart_values="0;0;0;0;0;0"     
        #return prefix+axis_values+cart_values
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        #['45','10'....]
    
        # calculate inverse kinematic solution
        transform = kin.transformation()
        transform.translate((float(values[0]), float(values[1]), float(values[2])))
        transform.rotate_x(np.deg2rad(float(values[3])))
        transform.rotate_y(np.deg2rad(float(values[4])))
        transform.rotate_z(np.deg2rad(float(values[5])))
        
        IKSol=transform.IKSolution()
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"

        # adding dummy values (you need to replace them with the solutions)
        #ik_values ="0;0;0;0;0;0"
        #print ik_values
        
        ik_values = [str(IKSol) for IKSol in transform.IKSolution()]
        print ik_values
        return prefix+"\n".join(ik_values)
        #return prefix+ik_values
    
    
if __name__ == "__main__":
    # setting up the operave environment
    env = openravepy.Environment() # create openrave environment @UndefinedVariable
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    env.SetViewer('qtcoin') # attach viewer (optional)
    robot = env.GetRobots()[0] # get the first robot
    dataTransfer()
    a=handleData("CAL")
    print a
    
    
