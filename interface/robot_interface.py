from interface.freedog.lowCmd import lowCmd
from interface.freedog.complex import motorCmd, motorCmdArray
from interface.freedog.enums import MotorModeLow
from interface.freedog.unitreeConnection import unitreeConnection, LOW_WIRED_DEFAULTS
from interface.freedog.lowState import lowState


class RobotInterface:
    def __init__(self):
        self.conn = unitreeConnection(LOW_WIRED_DEFAULTS)
        self.conn.startRecv()
        self.lcmd = lowCmd()
        self.lstate = lowState()
        self.mcmdarr = motorCmdArray()
        self.cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(self.cmd_bytes)

    def send_command(self, array):
        for motor_id in range(12):
            self.mcmdarr.setMotorCmd(motor_id, motorCmd(mode=0x0A, q=array[motor_id * 5],
                                                        dq=array[motor_id * 5 + 1], Kp=array[motor_id * 5 + 2],
                                                        Kd=array[motor_id * 5 + 3], tau=array[motor_id * 5 + 4]))
        self.lcmd.motorCmd = self.mcmdarr
        self.cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(self.cmd_bytes)

    def receive_observation(self):
        self.cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(self.cmd_bytes)
        state = self.conn.getData()
        return state
