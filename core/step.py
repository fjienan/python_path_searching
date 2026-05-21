class Step:
    """表示路径规划中的一个步骤"""
    def __init__(self, x, y, yaw, require_can_go=False, send_can_do=0):
        self.x=float(x)
        self.y=float(y)
        self.yaw=float(yaw)
        self.require_can_go = require_can_go
        self.send_can_do = send_can_do