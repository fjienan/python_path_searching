class Step:
    """表示路径规划中的一个步骤"""
    def __init__(self, x, y, yaw, require_can_go=False, send_can_do=False):
        self.x=x
        self.y=y
        self.yaw=yaw
        self.require_can_go = require_can_go
        self.send_can_do = send_can_do