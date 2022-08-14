from motor_group import *
from single_motor  import *
import sys
from PyQt5.QtCore import QThread, Qt
from PyQt5 import QtWidgets



# 與上位通訊使用物件
class CommandInterface(QThread):
    def __init__(self, name):
        super().__init__()
        self.status = 0 # 0: 手動控自 1:自動控自
        # 初始化身體以及馬達群組
        self.all_motors = dict()
        self.all_motors["NECK"] = MotorGroup("NECK")
        self.all_motors["BODY"] = MotorGroup("BODY")

        self.list_CommandRead = list()
        self.list_CommandWrite = list()

        # 啟動馬達群組通訊執行序
        self.all_motors["NECK"].start()
        self.all_motors["BODY"].start()
        
    # @brief 寫入讀取馬達資料資料array, paramter型態與ReadVp相同
    #
    # @param index ReadVP Read_inds序列
    #
    # @param type modbus address_r 資料
    #
    # @param data_len modbus byte_r 資料
    # 
    # @return
    def Set_Command_Read(self, index, type, data_len):
        self.list_CommandRead.append([index, type, data_len])

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
    def Set_Command_Write(self, index, goal_vel, goal_pos, type, data_len):
        self.list_CommandWrite.append([index, goal_vel, goal_pos, type, data_len])

    # @brief 取得馬達資料
    #
    # @param index 馬達IDvm4ux,4
    #
    # @param goal_vel modbus Goal_vels 資料
    # 
    # @return

    def Get_MotorData(self, index, type):
        # 判斷在哪個group中
        rtn_data = list()
        rtn_motor_list = []
        group_name = ''

        for key in self.all_motors["BODY"].motors:
            mot =  self.all_motors["BODY"].motors[key]
            rtn_motor_list.append(mot)

        for key in self.all_motors["NECK"].motors:
            mot =  self.all_motors["NECK"].motors[key]
            rtn_motor_list.append(mot)

        for i in range(0, len(rtn_motor_list)):
            print("{0}".format(rtn_motor_list[i].Read_vels))

        

        return rtn_motor_list

    

    def Set_PID_P(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_P(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_P(value)

    def Set_I(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_I(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_I(value)

    def Set_D(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_D(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_D(value)

    def Set_PWM(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_PWM(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_PWM(value)

    def Set_angle(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_angle(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_angle(value)
    def Set_vels(self, motor_id, value):
        if motor_id in self.all_motors["BODY"].motors:
            self.all_motors["BODY"].motors[motor_id].Set_vels(value)
        elif motor_id in self.all_motors["BODY"].motors:
            self.all_motors["NECK"].motors[motor_id].Set_vels(value)
    


    
    

    # @brief 讀取以及寫入所使用的執行緒
    #
    # @param
    # 
    # @return
    def run(self):
        while True:
            # 檢查尚未是否有下達讀取命令
            if len(self.list_CommandRead) != 0:
                tmp = self.list_CommandRead.pop(0)
                # 判斷在哪個group中
                if self.all_motors["NECK"].Check_WhetherInGroup(tmp[0]) == True:
                    self.all_motors["NECK"].Set_Command_ReadVP(tmp)
                elif self.all_motors["BODY"].Check_WhetherInGroup(tmp[0]) == True:
                    self.all_motors["BODY"].Set_Command_ReadVP(tmp)
            
            # 檢查尚未是否有下達寫入命令
            if len(self.list_CommandWrite) != 0:
                tmp = self.list_CommandWrite.pop(0)
                # 判斷在哪個group中
                if self.all_motors["NECK"].Check_WhetherInGroup(tmp[0]) == True:
                    self.all_motors["NECK"].Set_Command_WriteVP(tmp)
                elif self.all_motors["BODY"].Check_WhetherInGroup(tmp[0]) == True:
                    self.all_motors["BODY"].Set_Command_WriteVP(tmp)
            time.sleep(0.001)

if __name__=="__main__":
    app = QtWidgets.QApplication(sys.argv)
    ctrl_interface = CommandInterface("BODY")
    ctrl_interface.start() # 啟動與上位通訊主要執行序

    
    ctrl_interface.Get_MotorData([2,3], 10)



    '''
    print(ctrl_interface.all_motors["BODY"].motors[2].Get_Current_P())
    ctrl_interface.Set_PID_P(2, 20)
    print(ctrl_interface.all_motors["BODY"].motors[2].Get_Current_P())
    ctrl_interface.Set_vels(2, 20)
    '''
    
    print(ctrl_interface.all_motors["BODY"].motors[2].Get_Current_angle())
    ctrl_interface.Set_angle(2, 90)#輸入的單位是rpm
    print(ctrl_interface.all_motors["BODY"].motors[2].Get_Current_angle())

    print('ss start')
    sys.exit(app.exec_())
    #while True:
     #   print('end start')
     #  # print(ctrl_interface.Get_MotorData([2,3], 10))
     #   time.sleep(0.00001)
     



            


        

    


