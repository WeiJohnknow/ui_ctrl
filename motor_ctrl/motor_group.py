from PyQt5.QtCore import QThread, Qt
from PyQt5 import QtWidgets
from single_motor import *
from dynamixel_sdk import *
import numpy as np
import time

# 寫入個別port所使用的馬達ID
#BODY_MOTOR_ID_LIST = [1, 2, 3, 4, 5, 6, 7, 8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 , 16, 17, 18 ,19 ,20 ,21 ,22 ]
#NECK_MOTOR_ID_LIST = [38, 39]

BODY_MOTOR_ID_LIST = [2, 3]
NECK_MOTOR_ID_LIST = [1]

address_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
PROTOCOL_VERSION      = 2.0
BAUDRATE              = 3000000             
DEVICENAME_BODY            = 'COM9'  # 身體 modbus comport
DEVICENAME_NECK            = 'COM10' # 脖子以上 modbus comport  

MOTOR_CMD_HISTORY_COUNT = 10 #要記錄的歷史訊息資料數量

CYCLE_TIME = 40 # 單位為毫秒

def data2byte(data):
    return [DXL_LOBYTE(data), DXL_HIBYTE(data)]

def data4byte(data):
    return [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]

def data6byte(data,data1,data2):
    return data2byte(data) + data2byte(data2) + data2byte(data3)

def data8byte(data,data2):
    return data4byte(data) + data4byte(data2)

# 馬達群組化物件
class MotorGroup(QThread):
    def __init__(self, name):
        super().__init__()

        self.motors = dict() 
        self.DEVICENAME = ''  
        self.all_motor_id = []

        # 判斷為身體群組或者為頸部群組
        if name == "BODY":
             # 對頸部所使用的馬達物件初始化, 並加入dict中
            for index in BODY_MOTOR_ID_LIST:
                if index in self.motors:
                    continue
                else:
                    self.motors[index] = Motor(index)
            
            self.DEVICENAME = DEVICENAME_BODY
            self.all_motor_id = BODY_MOTOR_ID_LIST        
        elif name == "NECK":
            # 對頸部所使用的馬達物件初始化, 並加入dict中
            for index in NECK_MOTOR_ID_LIST:
                if index in self.motors:
                    continue
                else:
                    self.motors[index] = Motor(index)            
            self.DEVICENAME = DEVICENAME_NECK
            self.all_motor_id = NECK_MOTOR_ID_LIST      

        # 初始化 port 設定
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        # Open port
        self.portHandler.openPort()
        # Set port baudrate
        self.portHandler.setBaudRate(BAUDRATE)
        self.byte_r = 0
        self.adderss_r = 0 
        self.Data_cur_value = []
        self.Data_vel_value = []
        self.Data_pos_value = []
        self.Data_cur ={}
        self.Data_vel ={}
        self.Data_pos ={}
        self.Data_P_value = []
        self.Data_I_value = []
        self.Data_D_value = []
        self.Data_P ={}
        self.Data_I ={}
        self.Data_D ={}
        self.Data_PWM_value = []
        self.Data_PWM ={}
        self.Data_Unit_cur ={}
        self.Data_Unit_vel ={}

        self.list_CommandReadVP= []
        self.list_CommandWriteVP= []

    def Get_All_Motor_ID(self):
        rtn_id_list =()
        for m_id in self.motors.keys():
            rtn_id_list.append(m_id)
        
        return rtn_id_list
    
    def Set_Read_modbus(self, adderss_r, byte_r):
        self.byte_r = byte_r
        self.adderss_r = adderss_r
    # @brief 寫入馬達資料
    #
    # @param
    # 
    # @return
    def Set_Motor_Data(self):
        pass
    
    # @brief 寫入讀取馬達資料資料array, paramter型態與ReadVp相同
    #
    # @param index ReadVP Read_inds序列
    #
    # @param type modbus address_r 資料
    #
    # @param data_len modbus byte_r 資料
    # 
    # @return
    def Set_Command_ReadVP(self, index, type, data_len):
        self.list_CommandReadVP.append([index, type, data_len])

    
    # @brief 寫入讀取馬達資料資料array, paramter型態與ReadVp相同
    #
    # @param index Write_inds Read_inds序列
    #
    # @param goal_vel modbus Goal_vels 資料
    #
    # @param goal_pos modbus Goal_poss 資料
    #
    # @param type modbus address_w 資料
    #
    # @param data_len modbus byte_w 資料
    # 
    # @return
    def Set_Command_WriteVP(self, index, goal_vel, goal_pos, type, data_len):
        self.list_CommandWriteVP.append([index, goal_vel, goal_pos, type, data_len])

    # @brief 寫入讀取馬達資料資料array, paramter型態與ReadVp相同
    #
    # @param index Motor 索引序廖
    # 
    # @return True index 存在於該group
    # 
    # @return False index 不存在於該group
    def Check_WhetherInGroup(self, index):
        if len(index) <= 0:
            return False
        else:
            for i in index:
                if i in self.motors: 
                    return True


    # @brief 讀取馬達資料
    #
    # @param Read_inds index為馬達id
    #
    # @param address_r modbus 讀取 address
    #
    # @param byte_r modbus 讀取資料字節數
    # 
    # @return
    def ReadVP(self, Read_inds, address_r, byte_r):
        groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, address_r, byte_r)
        for i in range(len(Read_inds)):
            groupSyncRead.addParam(Read_inds[i])
        # 讀取PWM??
        if byte_r == 2:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                pwm_read= groupSyncRead.getData(Read_inds[i], address_r, 2)
                self.Data_PWM_value.append(pwm_read)
                if len(self.Data_PWM_value) >= 6:
                    Data_PWM_value.pop(0)
                self.Data_PWM[str(Read_inds[i])]=Data_PWM_value[-1]
        # 讀取PID參數
        elif byte_r == 6:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                D_read = groupSyncRead.getData(Read_inds[i], address_r, 2)
                I_read = groupSyncRead.getData(Read_inds[i], address_r+2, 2)
                p_read = groupSyncRead.getData(Read_inds[i], address_r+2, 2)
                self.Data_P_value.append(p_read)
                self.Data_I_value.append(I_read) 
                self.Data_D_value.append(D_read) 
                if len(self.Data_P_value) >= 6 and len(self.Data_I_value) >= 6 and len(self.Data_D_value) >= 6:
                    self.Data_P_value.pop(0)
                    self.Data_I_value.pop(0)
                    self.Data_D_value.pop(0)
                self.Data_P[str(Read_inds[i])]=Data_P_value[-1]
                self.Data_v[str(Read_inds[i])]=Data_I_value[-1]
                self.Data_p[str(Read_inds[i])]=Data_D_value[-1]

        # 讀取馬達電流, 速度, 位置
        elif byte_r == 10:
            groupSyncRead.txRxPacket()
            for i in range(len(Read_inds)):
                present_cur = groupSyncRead.getData(Read_inds[i], address_r, 2)
                present_vel = groupSyncRead.getData(Read_inds[i], address_r+2, 4)
                present_pos = groupSyncRead.getData(Read_inds[i], address_r+4, 4)
                #value
                self.Data_cur_value.append(present_cur)
                self.Data_vel_value.append(present_vel)
                self.Data_pos_value.append(present_pos)
                
                if len(self.Data_cur_value) >= 6 and len(self.Data_vel_value) >= 6 and len(self.Data_pos_value) >= 6:
                    self.Data_cur_value.pop(0)
                    self.Data_vel_value.pop(0)
                    self.Data_pos_value.pop(0)
                #key
                self.Data_cur[str(Read_inds[i])]=self.Data_cur_value[-1]
                self.Data_vel[str(Read_inds[i])]=self.Data_vel_value[-1]
                self.Data_pos[str(Read_inds[i])]=self.Data_pos_value[-1]  
            self.Data_Unit_cur = self.Unit_cur(Read_inds, self.Data_cur_value)
            self.Data_Unit_vel = self.Unit_vel(Read_inds, self.Data_vel_value)

    def WriteVP(self,Write_inds,Goal_vels,Goal_poss, address_w,byte_w):
        
        groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, address_w, byte_w)
        if byte_w == 2:
            for i in range(len(Write_inds)):
                vp = data2byte(PWM)
                groupSyncWrite.addParam(Write_inds[i],vp)
        elif byte_w == 6:
            for i in range(len(Write_inds)):
                vp = data2byte(D)+data2byte(I)+data2byte(P)
                groupSyncWrite.addParam(Write_inds[i],vp)   
        elif byte_w == 8:
            for i in range(len(Write_inds)):
                vp = data8byte(int(Goal_vels[Write_inds[i]]), int(Goal_poss[Write_inds[i]]))
                groupSyncWrite.addParam(Write_inds[i],vp)
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        #print('Write')
        
    def Unit_cur(self, Read_inds, Data_cur_value):
        Data_ch_cur =[]
        Data_Unit_cur = {}
        for i in range(len(Data_cur_value)):
            if (Data_cur_value[i]&(1<<15))>>15 == 1 :
                unit_cur=(65535-(Data_cur_value[i]-1))*0.00336
            elif (Data_cur_value[i]&(1<<15))>>15 == 0 :        
                unit_cur=Data_cur_value[i]*0.00336  
            Data_ch_cur.append(np.round(unit_cur,2))
            if len(Data_ch_cur) >= 6 :
                Data_ch_cur.pop(0)

        for i in range(len(Read_inds)):    
            Data_Unit_cur[str(Read_inds[i])]=Data_ch_cur
        return Data_Unit_cur

        
    def Unit_vel(self, Read_inds, Data_vel_value):
        Data_ch_vel =[]
        Data_Unit_vel = {}
        for i in range(len(Data_vel_value)):
            if (Data_vel_value[i]&(1<<31))>>31 == 1 :
                Unit_vel=(4294967296-(Data_vel_value[i]-1))                   
            elif (Data_vel_value[i]&(1<<31))>>31 == 0 :
                Unit_vel=Data_vel_value[i]                   
            Data_ch_vel.append(np.round(Unit_vel,2))
            if len(Data_ch_vel) >= 6 :
                Data_ch_vel.pop(0)
        for i in range(len(Read_inds)):   
            Data_Unit_vel[str(Read_inds[i])]=Data_ch_vel           
        return Data_Unit_vel
    
    def TORQUE_ON(self, Write_inds):
        for i in range(len(Write_inds)):
            self.packetHandler.write1ByteTxRx(self.portHandler, Write_inds[i], 64, 1)
            time.sleep(0.5)
            self.packetHandler.read1ByteRx(self.portHandler, Write_inds[i])
            time.sleep(0.5)
    def TORQUE_OFF(self, Write_inds):    
        for i in range(len(Write_inds)):
            self.packetHandler.write1ByteTxRx(self.portHandler, Write_inds[i], 64, 0)    

    # @brief 讀取以及寫入所使用的執行緒
    #
    # @param
    # 
    # @return
    def run(self):
        
        
        
        
        a = 0
        
        self.TORQUE_ON(self.all_motor_id)
        
        
        Goal_vels = np.ones(22)*20
        Goal_poss = np.ones(22)*2048
        while True:
            #print('start')
            time_start = time.time()
            # read motor data
            
            
            
            self.WriteVP(self.all_motor_id,Goal_vels,Goal_poss, 112,8)
            Goal_poss = np.ones(22)*a
            if a >= 3000:
                a = 0
            else:
                a+=100
            
            
            
            
            
            self.ReadVP(self.all_motor_id, 126, 10)
            for k in self.motors.keys(): # 接讀出來的數據寫入個別馬達物件中
                if k in self.motors:
                    self.motors[k].Add_vels(self.Data_Unit_vel[str(k)]) # 將讀出來的速度資料加入馬達物件中
                    self.motors[k].Add_angle(self.Data_Unit_vel[str(k)]) # 將讀出來的速度資料加入馬達物件中
                    #self.motors[k].Add_poss(self.Data_Unit_cur[str(k)]) # 將讀出來的位置資料加入馬達物件中
                    
                   # elif tmp_data[2] == 6: # 馬達 PID 資訊
                   #     self.motors[k].Set_P(self.Data_P[k])
                   #     self.motors[k].Set_I(self.Data_I[k])
                   #     self.motors[k].Set_D(self.Data_D[k])
                   # elif tmp_data[2] == 2: # 馬達 PWM??
                   #     self.motors[k].Set_PWM(self.Data_PWM[k])
                            
            # send command to motor
            if len(self.list_CommandWriteVP) != 0: # 有寫給馬達的訊息資料
                tmp_data = self.Set_Command_WriteVP.pop(0) # 取出第一筆資料
                self.WriteVP(tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3], tmp_data[4])
            
            
            # 限制每個時間週期都為 CYCLE_TIME 時間
            time_end = time.time()
            need_sleep_time = 0.0
            if abs(time_end - time_start) > abs(CYCLE_TIME/1000.0): # 總時間超過所設定的強制cycle time
                need_sleep_time = 0.0001 # sleep極小時間
            else:
                need_sleep_time = abs((CYCLE_TIME/1000.0) - abs(time_end - time_start))
            
            time.sleep(need_sleep_time)

#a = MotorGroup('BODY')
#a.start()
#print(a)

#while True:
#    time.sleep(0.01)