
MOTOR_HISTORY_COUNT = 10 #要記錄的歷史訊息資料數量

# 個別馬達物件
class Motor():
    def __init__(self, ID):
        super().__init__()
        self.ID = ID
        self.vels = 20
        self.Now_vels = []
        self.Now_poss = []
        self.Now_angle = []
        self.angle = 0
        self.poss = 2048
        self.PWM = 885
        self.P = 850
        self.I = 0
        self.D = 0

    # @brief 設定馬達PID P
    #
    # @param P PID P設定值
    # 
    # @return 
    def Set_P(self, P):
        self.P = P
    
    # @brief 設定馬達PID I
    #
    # @param I PID I設定值
    # 
    # @return
    def Set_I(self, I):
        self.I = I
        
    # @brief 設定馬達PID D
    #
    # @param D PID D設定值
    # 
    # @return
    def Set_D(self, D):
        self.D = D

    def Set_PWM(self, PWM):
        self.PWM = PWM
 
    def Set_vels(self, vels):
        self.vels = vels/0.229
        
    
    def Set_angle(self, angle):
        self.poss = 2048+(angle*4095)/360

    # @brief 回傳當前 PID P data 
    #
    # @param 
    # 
    # @return self.P
    def Get_Current_P(self):
        return self.P

    # @brief 回傳當前 PID I data
    #
    # @param 
    # 
    # @return self.I
    def Get_Current_I(self):
        return self.I

    # @brief 回傳當前 PID D data
    #
    # @param 
    # 
    # @return self.D
    def Get_Current_D(self):
        return self.D

    def Get_Current_angle(self):
        self.angle = (self.poss/4095)*360
        return self.angle

    def Get_Current_vels(self):
        self.vels = self.vels*0.229
        return self.vels
    # @brief 將讀取出來的馬達資料加入數據中 
    #
    # @param data 馬達數據資料
    # 
    # @return
    def Add_vels(self, data):     
        # 判斷資料數量是否超出設定的歷史紀錄數量
        if len(self.Now_vels) >= MOTOR_HISTORY_COUNT: 
            self.Now_vels.pop(0) # 移除第一筆資料
            self.Now_vels.append(data)
        else:
            self.Now_vels.append(data)

    # @brief 將讀取出來的馬達資料加入數據中 
    #
    # @param data 馬達位置數據資料
    # 
    # @return
    def Add_poss(self, data):
        # 判斷資料數量是否超出設定的歷史紀錄數量
        if len(self.Now_poss) >= MOTOR_HISTORY_COUNT: 
            self.Now_poss.pop(0) # 移除第一筆資料
            self.Now_poss.append(data)
        else:
            self.Now_poss.append(data)
        # 把Now_poss容器中的數值逐一調出來做換算
        self.Now_angle.clear()
        for i in range(0, len(self.Now_poss)):
            tmp_angle = (self.Now_poss[i] / 4095.0) / 360.0
            self.Now_angle.append(tmp_angle)
            
    def Add_angle(self, data):
        # 判斷資料數量是否超出設定的歷史紀錄數量      
        if len(self.Now_angle) >= MOTOR_HISTORY_COUNT: 
            self.Now_angle.pop(0) # 移除第一筆資料
            self.Now_angle.append(data)
        else:
            self.Now_angle.append(data)
    # @brief 回傳馬達速度歷史數據資料
    #
    # @param 
    # 
    # @return rtn_vel 速度 history data
    def Read_vels(self):
        rtn_vel = self.Now_vels.copy() 
        return rtn_vel
    
    def Read_angle(self):
        rtn_angle = self.Now_angle.copy() 
        return rtn_angle    
    
    # @brief 回傳馬達位置歷史數據資料
    #
    # @param 
    # 
    # @return rtn_vel 位置 history data
    def Read_poss(self):
        rtn_vel = self.poss.copy() 
        return  rtn_vel



