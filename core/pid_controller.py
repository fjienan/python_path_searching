class PIDController:
    """离散时间 PID 控制器"""
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_timestamp = None

    def reset(self):
        """重置积分项和上一次误差（切换目标点时调用）"""
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_timestamp = None

    def compute(self, current_error, dt):
        """计算 PID 控制量"""
        if dt <= 0.0:
            return 0.0

        self.integral_error += current_error * dt

        if self.output_limit is not None and self.ki != 0.0:
            i_limit = self.output_limit / self.ki
            self.integral_error = max(-i_limit, min(i_limit, self.integral_error))

        derivative_error = (current_error - self.last_error) / dt
        self.last_error = current_error

        output = self.kp * current_error + self.ki * self.integral_error + self.kd * derivative_error

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        return output
