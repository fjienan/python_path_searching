class PIDController:
    """简单的离散时间 PID 控制器"""
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral_error = 0.0
        self.last_error = 0.0
        self.output_limit = output_limit

    def reset(self):
        """重置内部状态"""
        self.integral_error = 0.0
        self.last_error = 0.0

    def compute(self, current_error, dt):
        """计算最新的控制量"""
        if dt <= 0.0:
            return 0.0
            
        self.integral_error += current_error * dt
        
        # 简单积分抗饱和限幅
        if self.output_limit is not None:
            # 近似地认为积分项不超过总输出限制
            self.integral_error = max(-self.output_limit/self.ki if self.ki != 0 else 0, 
                                      min(self.output_limit/self.ki if self.ki != 0 else 0, self.integral_error))
                                      
        derivative_error = (current_error - self.last_error) / dt
        self.last_error = current_error
        
        output = self.kp * current_error + self.ki * self.integral_error + self.kd * derivative_error
        
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
            
        return output
