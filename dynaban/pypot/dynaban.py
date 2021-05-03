import time
import pypot.robot
import pypot.dynamixel

'''
home_pos = [90,0,0,0]
'''
class dynaban:
    def __init__(self, motor_id, gripper_id):
        self.home_pos = [90,0,0,0]
        self.OPEN_LIMIT = 150 # in degrees for pypot, 1023 in steps (Max angle for AX-12A)
        self.CLOSE_LIMIT = 100 #81.96 # in degrees for pypot, 791 in steps
        self.init_pos = False
        self.MOTOR_ID = motor_id
        self.GRIPPER_ID = gripper_id
        self.DXL_DICT_3      = dict(zip(self.MOTOR_ID, [3]*len(self.MOTOR_ID)))
        self.DXL_DICT_1      = dict(zip(self.MOTOR_ID, [1]*len(self.MOTOR_ID)))
        self.ports = pypot.dynamixel.get_available_ports()
        
        if not self.ports:
            raise IOError('no port found!')

        print('ports found', self.ports)

        print('connecting on the first available port:', self.ports[0])

        self.dxl_io = pypot.dynamixel.DxlIO(self.ports[0])
        self.dxl_io.enable_torque({self.GRIPPER_ID: 1})
        time.sleep(0.1)    
    def setTraj1(self, id, duration, coeffs):
        errorCounter = 0
        delay = 0.001
        while True:
            try:
                self.dxl_io.set_traj1_size({id: 4})
                time.sleep(delay)
                self.dxl_io.set_duration1({id: duration})
                time.sleep(delay)
                self.dxl_io.set_a0_traj1({id: coeffs[0]})
                time.sleep(delay)
                self.dxl_io.set_a1_traj1({id: coeffs[1]})
                time.sleep(delay)
                self.dxl_io.set_a2_traj1({id: coeffs[2]})
                time.sleep(delay)
                self.dxl_io.set_a3_traj1({id: coeffs[3]})
                time.sleep(delay)
    #             self.dxl_io.set_a4_traj1({id: coeffs[4]})
    #             time.sleep(delay)

                break
            except:
                errorCounter = errorCounter + 1
                # print "Nope :/"
                break
                print("nb errors1 = ", errorCounter)


    def setTraj2(self,id, duration, coeffs):
        errorCounter = 0
        delay = 0.001

        while True:
            try:
                self.dxl_io.set_traj2_size({id: 4})
                time.sleep(delay)
                self.dxl_io.set_duration2({id: duration})
                time.sleep(delay)
                self.dxl_io.set_a0_traj2({id: coeffs[0]})
                time.sleep(delay)
                self.dxl_io.set_a1_traj2({id: coeffs[1]})
                time.sleep(delay)
                self.dxl_io.set_a2_traj2({id: coeffs[2]})
                time.sleep(delay)
                self.dxl_io.set_a3_traj2({id: coeffs[3]})
                time.sleep(delay)
    #             self.dxl_io.set_a4_traj2({id: coeffs[4]})
    #             time.sleep(delay)

                break
            except:
                errorCounter = errorCounter + 1
                print("nb errors2 = ", errorCounter)
                break


    def setTorque1(self,id, duration, coeffs):
        errorCounter = 0
        delay = 0.001
        while True:
            try:
                self.dxl_io.set_torque1_size({id: 4})
                time.sleep(delay)
                self.dxl_io.set_duration1({id: duration})
                time.sleep(delay)
                self.dxl_io.set_a0_torque1({id: coeffs[0]})
                time.sleep(delay)
                self.dxl_io.set_a1_torque1({id: coeffs[1]})
                time.sleep(delay)
                self.dxl_io.set_a2_torque1({id: coeffs[2]})
                time.sleep(delay)
                self.dxl_io.set_a3_torque1({id: coeffs[3]})
                time.sleep(delay)
    #             self.dxl_io.set_a4_torque1({id: coeffs[4]})
    #             time.sleep(delay)
                break
            except:
                errorCounter = errorCounter + 1
                # print "Nope :/"
                pass
    #         print "Nb errors : ", errorCounter

    def setTorque2(self,id, duration, coeffs):
        errorCounter = 0
        delay = 0.001
        while True:
            try:
                self.dxl_io.set_torque2_size({id: 4})
                time.sleep(delay)
                self.dxl_io.set_duration2({id: duration})
                time.sleep(delay)
                self.dxl_io.set_a0_torque2({id: coeffs[0]})
                time.sleep(delay)
                self.dxl_io.set_a1_torque2({id: coeffs[1]})
                time.sleep(delay)
                self.dxl_io.set_a2_torque2({id: coeffs[2]})
                time.sleep(delay)
                self.dxl_io.set_a3_torque2({id: coeffs[3]})
                time.sleep(delay)
    #             self.dxl_io.set_a4_torque2({id: coeffs[4]})
    #             time.sleep(delay)
                break
            except:
                errorCounter = errorCounter + 1
                # print "Nope :/"
                pass
            
    def init_motor(self):
                
        for joints in range(len(self.MOTOR_ID)):
            
            num = self.MOTOR_ID[joints]
            self.dxl_io.set_mode_dynaban({num:0})
            time.sleep(0.1)
            self.dxl_io.enable_torque({num:1})
            time.sleep(0.1)
            self.dxl_io.set_pid_gain({self.MOTOR_ID[joints]:[1,0,0]})
            time.sleep(0.1)
            
    def set_zero(self):
        diff = []
        for joints in range(len(self.MOTOR_ID)):
            num = self.MOTOR_ID[joints]
            current_pos = int(self.dxl_io.get_present_position([num])[0])
            diff.append(abs(current_pos - self.home_pos[joints]))
#             print(range(diff))
            for i in range(diff[joints]):
                if (current_pos > 0):
                    self.dxl_io.set_goal_position({self.MOTOR_ID[joints]:(current_pos -i)})
                    time.sleep(0.01) 
                else:
                    self.dxl_io.set_goal_position({self.MOTOR_ID[joints]:(current_pos + i)})
                    time.sleep(0.01)

    def go_to_start_pos(self,coeff_angle):
        
        for joints in range(len(self.MOTOR_ID)):
            num = self.MOTOR_ID[joints]
            start_angle = int(coeff_angle[joints][3]*360/4096-180)
            print(start_angle)
            current_pos = int(self.dxl_io.get_present_position([num])[0])
            diff = abs(current_pos-start_angle)
            for i in range(diff):
                if (current_pos > start_angle):
                    self.dxl_io.set_goal_position({self.MOTOR_ID[joints]:current_pos - i})
                    time.sleep(0.01) 
                else:
                    self.dxl_io.set_goal_position({self.MOTOR_ID[joints]:current_pos + i})
                    time.sleep(0.01)

            time.sleep(0.5)
        

    def write_to_motor(self,coeff_angle, spline ,coeff_torque = []):
        if not self.init_pos:
            for joints in range(len(self.MOTOR_ID)):
                
                print("traj1 ", joints)
                self.setTraj1(self.MOTOR_ID[joints],spline, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])
                
                if coeff_torque != []:
                    self.setTorque1(self.MOTOR_ID[joints],spline, [coeff_torque[joints][3],coeff_torque[joints][2],coeff_torque[joints][1],coeff_torque[joints][0]])

            self.dxl_io.set_mode_dynaban(self.DXL_DICT_3)
            print("set mode")
            self.init_pos = True

        else:
            for joints in range(len(self.MOTOR_ID)):
                print("traj2 ", joints)
                self.setTraj2(self.MOTOR_ID[joints],spline, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])

                if coeff_torque != []:
                    self.setTorque2(self.MOTOR_ID[joints],spline, [coeff_torque[joints][3],coeff_torque[joints][2],coeff_torque[joints][1],coeff_torque[joints][0]])

            
            self.dxl_io.set_copy_next_buffer(self.DXL_DICT_1)
            print("set buffer")

    def open_gripper(self):
        start = self.dxl_io.get_present_position([self.GRIPPER_ID])[0]
        cur = int(start)
        while(cur < self.OPEN_LIMIT):
            cur = cur + 1
            self.dxl_io.set_goal_position({self.GRIPPER_ID: cur})
            time.sleep(0.01)

#         print("[+]Gripper opened")
#         print(self.dxl_io.get_present_position([self.GRIPPER_ID]))

    def close_gripper(self):
        start = self.dxl_io.get_present_position([self.GRIPPER_ID])[0]
        cur = int(start)
        while(cur > self.CLOSE_LIMIT):
            cur = cur - 1
            self.dxl_io.set_goal_position({self.GRIPPER_ID: cur})
            time.sleep(0.01)
#         print("[+]Gripper closed")
#         print(self.dxl_io.get_present_position([self.GRIPPER_ID]))

    

# if _name=="main_":