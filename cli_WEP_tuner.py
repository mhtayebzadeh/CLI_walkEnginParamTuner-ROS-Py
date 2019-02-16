#!/usr/bin/env python
import os
import sys
import readline
import glob
import rospy
from kinematic_pkg.msg import WEP_msg
from std_msgs.msg import Empty
import time
import datetime
import pickle
import copy

log_file_name = "CLI_WEP_tuner.log"

file_path = __file__
directory = os.path.dirname(file_path)
dir_path = os.path.join(directory ,"WEP_Records")
log_file_path = os.path.join(dir_path , log_file_name)
if not os.path.exists(dir_path):
    os.makedirs(dir_path)


Current_WEP_on_robot = [0 for i in range(101)]


def readCurrentWEPsCallback(msg):
    global Current_WEP_on_robot
    for i in range(msg.sizeOfArr):
        Current_WEP_on_robot[msg.index[i]] = msg.WEP[i]


def LogAllCurrentWEPs():
    WEP_NUM = 101
    global Current_WEP_on_robot
    global log_file_path
    global indexDict
    os.system(
        "echo " + " \"<===============================================>\"  >> " + log_file_path)
    os.system(
        "echo " + " \"<===============================================>\"  >> " + log_file_path)
    for i in range(WEP_NUM):
        log_str = "WEP[" + indexDict[i] + "(" + str(i) + ")] = " + \
            str(Current_WEP_on_robot[i]) + " on the robot "
        log_time = unicode(datetime.datetime.now())
        print log_str
        os.system("echo " + "\"at ["+log_time+"]    " +
                  log_str + "\" >> " + log_file_path)
    os.system(
        "echo " + " \"<===============================================>\"  >> " + log_file_path)
    os.system(
        "echo " + " \"<===============================================>\"  >> " + log_file_path)

    time.sleep(2)


def clearLogs():
    global log_file_path
    os.system("echo " + "   > " + log_file_path)


def read_WEP(paramNum):
    WEP_NUM = 101
    global Current_WEP_on_robot
    global log_file_path
    global indexDict
    os.system(
        "echo " + " \"===================================\"  >> " + log_file_path)

    log_str = "WEP[" + indexDict[paramNum] + "(" + str(paramNum) + ")] = " + \
            str(Current_WEP_on_robot[paramNum]) + " on the robot "
    log_time = unicode(datetime.datetime.now())
    print log_str
    os.system("echo " + "\"at ["+log_time+"]    " +
              log_str + "\" >> " + log_file_path)
    os.system(
        "echo " + " \"===================================\"  >> " + log_file_path)
    time.sleep(0.5)


def send_All_current_WEP():
    global Current_WEP_on_robot
    global log_file_path
    global indexDict
    msg = WEP_msg()
    WEP_NUM = 101
    msg.sizeOfArr = WEP_NUM
    msg.WEP = [0 for i in range(WEP_NUM)]
    msg.index = [0 for i in range(WEP_NUM)]
    os.system("echo " + " \"<++++++++++++++++++++++++++++++++++++++++++++++++>\"  >> " + log_file_path)
    for i in range(WEP_NUM):
        msg.WEP[i] = Current_WEP_on_robot[i]
        msg.index[i] = i
        log_str = "WEP[" + indexDict[i] + "(" + str(i) + ")] updated to " + \
            str(Current_WEP_on_robot[i]) + " "
        log_time = unicode(datetime.datetime.now())
        print log_str
        os.system("echo " + "\"at ["+log_time+"]    " + log_str + "\" >> " + log_file_path)
    
    os.system("echo " + " \"<++++++++++++++++++++++++++++++++++++++++++++++++>\"  >> " + log_file_path)
    pub_WEP.publish(msg)


def send_WEP(paramIndex, value):
    global indexDict
    msg = WEP_msg()
    msg.sizeOfArr = 1
    WEP_NUM = 101
    msg.WEP = [0 for i in range(WEP_NUM)]
    msg.index = [0 for i in range(WEP_NUM)]

    msg.WEP[0] = value
    msg.index[0] = paramIndex
    pub_WEP.publish(msg)

    log_str = "WEP[" + indexDict[paramIndex] + "(" + str(paramIndex) + ")] updated to " + \
        str(value) + " "
    log_time = unicode(datetime.datetime.now())
    print log_str
    os.system("echo " + "\"at ["+log_time+"]    " +
              log_str + "\" >> " + log_file_path)
    # time.sleep(2)


def makeParamDict():
    myDict = {}
    # Walk Engine Parameters
    # motion resoulotion (min=0.001  max=0.1)
    myDict[" "] = 0
    myDict["P_Motion_Resolution"] = 1
    myDict["P_Gait_Frequency"] = 2  # gait frequency (min=0.001  max=1.0)
    # double support sleep (min=0.001  max=1.0)
    myDict["P_Double_Support_Sleep"] = 3
    # single support sleep (min=0.001  max=1.0)
    myDict["P_Single_Support_Sleep"] = 4
    myDict["P_Fly_Roll_Gain"] = 5  # fly leg roll gain
    myDict["P_Fly_Pitch_Gain"] = 6  # fly leg pitch gain
    myDict["P_Fly_Yaw_Gain"] = 7  # fly leg yaw gain
    myDict["P_Fly_X_Swing_Gain"] = 8
    myDict["P_Fly_Y_Swing_Gain"] = 9
    # fly leg step height gain (min=0.001  max=1.0)
    myDict["P_Fly_Z_Swing_Gain"] = 10
    myDict["P_Support_Roll_Gain"] = 11  # support leg roll gain
    myDict["P_Support_Pitch_Gain"] = 12  # support leg pitch gain
    myDict["P_Support_Yaw_Gain"] = 13  # support leg yaw gain
    myDict["P_Support_X_Swing_Gain"] = 14
    myDict["P_Support_Y_Swing_Gain"] = 15  # support leg Y gain (sideward)
    # support leg Z gain (topdown) this parameter also can use for push
    myDict["P_Support_Z_Swing_Gain"] = 16
    myDict["P_Body_X_Swing_Gain"] = 17
    # body sideward swing gain (for swing both of legs in y axis during walk)
    myDict["P_Body_Y_Swing_Gain"] = 18
    # body topdown swing gain (for swing both of legs in Z axis during walk)
    myDict["P_Body_Z_Swing_Gain"] = 19

    # stablization parameters
    myDict["P_Stablizer_Arm_Pitch_Gain"] = 20  # add
    myDict["P_Stablizer_Arm_Roll_Gain"] = 21  # add
    myDict["P_Stablizer_Arm_Elbow_Gain"] = 22
    myDict["P_Stablizer_Hip_Roll_Gain"] = 23  # add
    myDict["P_Stablizer_Hip_Pitch_Gain"] = 24  # add
    myDict["P_Stablizer_Knee_Gain"] = 25  # add
    myDict["P_Stablizer_Foot_Pitch_Gain"] = 26  # add
    myDict["P_Stablizer_Foot_Roll_Gain"] = 27  # add
    myDict["P_Stablizer_COM_X_Shift_Gain"] = 28  # add
    myDict["P_Stablizer_COM_Y_Shift_Gain"] = 29  # add

    myDict["P_Gyro_Stablizer_Arm_Pitch_Gain"] = 30  # add
    myDict["P_Gyro_Stablizer_Arm_Roll_Gain"] = 31
    myDict["P_Gyro_Stablizer_Arm_Elbow_Gain"] = 32
    myDict["P_Gyro_Stablizer_Hip_Roll_Gain"] = 33  # add
    myDict["P_Gyro_Stablizer_Hip_Pitch_Gain"] = 34  # add
    myDict["P_Gyro_Stablizer_Knee_Gain"] = 35  # add
    myDict["P_Gyro_Stablizer_Foot_Pitch_Gain"] = 36  # add
    myDict["P_Gyro_Stablizer_Foot_Roll_Gain"] = 37  # add
    myDict["P_Gyro_Stablizer_COM_X_Shift_Gain"] = 38  # add
    myDict["P_Gyro_Stablizer_COM_Y_Shift_Gain"] = 39  # add

    # hopping gait gain
    myDict["P_Stablizer_Hopping_Gait_X_Gain"] = 40
    myDict["P_Stablizer_Hopping_Gait_Y_Gain"] = 41

    # both leg offset in inverse kinematic (body COM)
    myDict["P_COM_X_offset"] = 42  # add
    myDict["P_COM_Y_offset"] = 43  # add
    myDict["P_COM_Z_offset"] = 44  # add
    myDict["P_COM_Roll_offset"] = 45  # add
    myDict["P_COM_Pitch_offset"] = 46  # add
    myDict["P_COM_Yaw_offset"] = 47  # add

    # legs joints offset
    myDict["P_Left_Leg_Hip_Yaw_Offset"] = 48  # add
    myDict["P_Left_Leg_Hip_Roll_Offset"] = 49  # add
    myDict["P_Left_Leg_Hip_Pitch_Offset"] = 50  # add
    myDict["P_Left_Leg_Knee_Offset"] = 51  # add
    myDict["P_Left_Leg_Foot_Pitch_Offset"] = 52  # add
    myDict["P_Left_Leg_Foot_Roll_Offset"] = 53  # add

    myDict["P_Right_Leg_Hip_Yaw_Offset"] = 54  # add
    myDict["P_Right_Leg_Hip_Roll_Offset"] = 55  # add
    myDict["P_Right_Leg_Hip_Pitch_Offset"] = 56  # add
    myDict["P_Right_Leg_Knee_Offset"] = 57  # add
    myDict["P_Right_Leg_Foot_Pitch_Offset"] = 58  # add
    myDict["P_Right_Leg_Foot_Roll_Offset"] = 59  # add

    # Left leg inverse kinematic offset
    myDict["P_Left_Leg_X_Offset"] = 60  # add
    myDict["P_Left_Leg_Y_Offset"] = 61  # add
    myDict["P_Left_Leg_Z_Offset"] = 62  # add
    myDict["P_Left_Leg_Roll_Offset"] = 63  # add
    myDict["P_Left_Leg_Pitch_Offset"] = 64  # add
    myDict["P_Left_Leg_Yaw_Offset"] = 65  # add

    myDict["P_Right_Leg_X_Offset"] = 66  # add
    myDict["P_Right_Leg_Y_Offset"] = 67  # add
    myDict["P_Right_Leg_Z_Offset"] = 68  # add
    myDict["P_Right_Leg_Roll_Offset"] = 69  # add
    myDict["P_Right_Leg_Pitch_Offset"] = 70  # add
    myDict["P_Right_Leg_Yaw_Offset"] = 71  # add

    myDict["P_R_Arm_Pitch_offset"] = 72  # add
    myDict["P_R_Arm_Roll_offset"] = 73  # add
    myDict["P_R_Arm_Elbow_offset"] = 74  # add

    myDict["P_L_Arm_Pitch_offset"] = 75  # add
    myDict["P_L_Arm_Roll_offset"] = 76  # add
    myDict["P_L_Arm_Elbow_offset"] = 77  # add

    # fall thershold
    myDict["P_Fall_Roll_Thershold"] = 78
    myDict["P_Fall_Pitch_Thershold"] = 79

    # imu offset
    myDict["P_IMU_X_Angle_Offset"] = 80  # add
    myDict["P_IMU_Y_Angle_Offset"] = 81  # add

    # MPU filtering parametrs
    myDict["P_Gyro_X_LowPass_Gain"] = 82  # add
    myDict["P_Gyro_Y_LowPass_Gain"] = 83  # add

    # kalman filter r mesurement value
    myDict["P_Kalman_Roll_RM_Rate"] = 84
    myDict["P_Kalman_Pitch_RM_Rate"] = 85
    myDict["P_Kalman_Yaw_RM_Rate"] = 86

    # smoothing ratio
    myDict["P_Vx_Smoothing_Ratio"] = 87
    myDict["P_Vy_Smoothing_Ratio"] = 88
    myDict["P_Vt_Smoothing_Ratio"] = 89

    myDict["P_Leg_Length"] = 90  # add

    myDict["P_Head_Pan_Speed"] = 91
    myDict["P_Head_Tilt_Speed"] = 92

    myDict["P_Min_Voltage_Limit"] = 93

    myDict["Vx_Offset"] = 94
    myDict["Vy_Offset"] = 95
    myDict["Vt_Offset"] = 96

    myDict["P_Left_Leg_Hip_Pitch_Offset_Original"] = 97  # add
    myDict["P_Right_Leg_Hip_Pitch_Offset_Original"] = 98  # add
    myDict["P_Left_Leg_Hip_Pitch_Offset_Backwards"] = 99  # add
    myDict["P_Right_Leg_Hip_Pitch_Offset_Backwards"] = 100  # add

    myDict["WEP_NUM"] = 101

    return myDict


def save_current_WEP(filename):
    global dir_path
    global Current_WEP_on_robot
    file = os.path.join(dir_path , filename + ".pckl")
    f = open(file, 'wb')
    pickle.dump(Current_WEP_on_robot, f)
    f.close()
    print "WEPs saved in " + str(file)
    time.sleep(1.5)

def load_current_WEP(filename):
    global dir_path
    global Current_WEP_on_robot
    file = os.path.join(dir_path , filename + ".pckl")
    f = open(file, 'rb')
    obj = pickle.load(f)
    Current_WEP_on_robot = copy.copy(obj)
    f.close()
    print "WEPs loaded from " + str(file)
    time.sleep(1.5)


def Help():
    global log_file_path
    print """
    welcom to WEP_tuner AUTman robot ...

    log file path : """ + log_file_path + """
    
    Input [paramName(or param index) _space_ value] to update param on robot.
    Input [paramName(or param index) _space_ ?] to read param from robot.
    Input [all _space_ ?] to read All params from robot.
    Input [all _space_ .] to set All current params to robot.

    Input [save __filename__] to save All current params to \{dir\}/WEP_Records/filename.pckl .
    Input [load __filename__] to load All saved params to current params (not applied to robot) .

    Input [clear] to clear log file.
    Input [end] to Exit.
    
    github.com/mhtayebzadeh   
    AUTcup 2019
    """


class tabCompleter(object):
    """ 
    A tab completer that can either complete from
    the filesystem or from a list.

    Partially taken from:
    http://stackoverflow.com/questions/5637124/tab-completion-in-pythons-raw-input
    """

    def pathCompleter(self, text, state):
        """ 
        This is the tab completer for systems paths.
        Only tested on *nix systems
        """
        line = readline.get_line_buffer().split()

        return [x for x in glob.glob(text+'*')][state]

    def createListCompleter(self, ll):
        """ 
        This is a closure that creates a method that autocompletes from
        the given list.

        Since the autocomplete function can't be given a list to complete from
        a closure is used to create the listCompleter function with a list to complete
        from.
        """
        def listCompleter(text, state):
            line = readline.get_line_buffer()

            if not line:
                return [c + " " for c in ll][state]

            else:
                return [c + " " for c in ll if c.startswith(line)][state]

        self.listCompleter = listCompleter


try:
    pub_WEP = rospy.Publisher('update_WEP', WEP_msg, queue_size=10)
    pub_empty = rospy.Publisher('read_WEP', Empty, queue_size=10)
    rospy.Subscriber("readCurrentWEP", WEP_msg, readCurrentWEPsCallback)
    rospy.init_node('cmd_WEP_updater', anonymous=True)
    empty_msg = Empty()
except:
    print "ROS failed!"
    print "you should run \"roscore\" command first!  "
    sys.exit()

try:
    pub_empty.publish(empty_msg)
except:
    print "Robot ROS node not connected!"

if __name__ == "__main__":
    paramDict = makeParamDict()
    indexDict = {}
    for i in paramDict:
        indexDict[paramDict[i]] = i
    paramList = [i for i in paramDict]
    paramList.append("clear")
    paramList.append("all")
    paramList.append("end")
    paramList.append("save")
    paramList.append("load")

    # print paramList
    t = tabCompleter()
    t.createListCompleter(paramList)

    readline.set_completer_delims('\t')
    readline.parse_and_bind("tab: complete")

    readline.set_completer(t.listCompleter)

    while True:
        os.system("clear")
        ans = raw_input(
            "Input \"end\" for Exit or input help for Help.\nInput_WEP_name(or index)   value :\n")
        # print ans
        if (str(ans) == "end" or str(ans) == "e"):
            break
        elif (str(ans) == "help"):
            Help()
            time.sleep(1.5)
            ans = raw_input("Press Enter to continue ...\n")
            continue
        elif (str(ans) == "clear" or str(ans) == "clear log" or str(ans) =="cl"):
            clearLogs()
            continue
        elif (str(ans) == "save"):
            save_current_WEP("current_WEP")
        elif (str(ans) == "load"):
            load_current_WEP("current_WEP")
        else:
            ans = ans.split(' ')
            try:
                if (ans[0] == "" or ans[0] == " "):
                    param = ans[1]
                    value = ans[2]
                else:
                    param = ans[0]
                    value = ans[1]

                if (param == "save"):
                    save_current_WEP(value)
                    continue
                elif (param == "load"):
                    load_current_WEP(value)

                if value == "?":
                    pub_empty.publish(empty_msg)
                    time.sleep(0.5)
                    if param == "All" or param == "all":
                        LogAllCurrentWEPs()
                    else:
                        if (param in paramList):
                            read_WEP(int(paramDict[param]))
                        elif (int(param) < 101):
                            read_WEP(int(param))
                        else:
                            print "input is not valid....."
                else:
                    if (param == "all" or param == "All"):
                        if (value == "." or value == "*" or value == "set"):
                            send_All_current_WEP()

                    elif (param in paramList):
                        send_WEP(paramDict[param], float(value))
                    elif (int(param) < 101):
                        send_WEP(int(param), float(value))
                    else:
                        print "input is not valid"

                time.sleep(1)

            except:
                print "input is not valid ----"
                time.sleep(1)
        time.sleep(0.5)