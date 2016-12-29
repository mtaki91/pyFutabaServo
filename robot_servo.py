import serial
import struct
from time import sleep

class serial_converter(serial.Serial):
  def __init__(self, Port, Baudrate):
    super(serial_converter, self).__init__()
    self.port = Port
    self.baudrate = Baudrate
    self.parity = serial.PARITY_NONE
    self.stopbits = serial.STOPBITS_ONE
    self.timeout = 1
    self.connect()
    
  def connect(self):
    try:
      self.open()
      self.isOpen = True
      self.isBusy = False
      return True

    except:
      self.isOpen = False
      self.isBusy = False
      return False


class futaba_servo:
  def __init__(self, serial_conv, servo_id):
    self._serial_conv = serial_conv
    self._id = servo_id

  def is_open(self):
    return self._serial_conv.isOpen

  def _send_command(self, Flag, Address, Length, Count, Data=[]):
    command = [0xFA, 0xAF, self._id]

    for tmp in [Flag, Address, Length, Count, Data]:
      if isinstance(tmp, int):
        command.append(tmp)
        
      elif isinstance(tmp, list):
        command = command + tmp

      else:
        raise ValueError()

    Sum = command[2]

    for tmp in command[3:]:
      Sum = Sum ^ tmp
      
    command.append(Sum)
    
    for tmp in command:
      self._serial_conv.write(struct.pack('B', tmp))
      sleep(0.001)
   
  def move(self, Position, Time):
    data = [Position & 0x00FF, (Position & 0xFF00) >> 8 , Time & 0x00FF, (Time & 0xFF00) >> 8 ]

    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._send_command(0x00, 0x1E, 0x04, 0x01, data)
    self._serial_conv.isBusy = False
  
  def torque(self, Mode):
    """
    Set Servo's torque
    Mode = 0 : Off
         = 1 : On
         = 2 : Brake
    """

    data = Mode & 0x00FF

    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._send_command(0x00, 0x24, 0x01, 0x01, data)
    self._serial_conv.isBusy = False

  def get_status(self):
    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._send_command(0x09, 0x00, 0x00, 0x01)

    readbuf = []
    for x in range(26):
      readbuf.append(self._serial_conv.read())

    self._serial_conv.isBusy = False

    _angle = struct.unpack('h', readbuf[7] + readbuf[8])[0]
    _time = struct.unpack('h', readbuf[9] + readbuf[10])[0]
    _speed = struct.unpack('h', readbuf[11] + readbuf[12])[0]
    _load = struct.unpack('h', readbuf[13] + readbuf[14])[0]
    _temperature = struct.unpack('h', readbuf[15] + readbuf[16])[0]
    _voltage = struct.unpack('h', readbuf[17] + readbuf[18])[0]

    return {"Angle": _angle,           
            "Time": _time,
            "Speed": _speed,
            "Load": _load,
            "Temperature": _temperature,
            "Voltage": _voltage}


class robot_servos:
  def __init__(self, servo_list, activate_sequence):
    self.servo_list = servo_list
    self._num_servo = len(servo_list)
    self._activate_sequence = activate_sequence

  def is_open(self):
    for servo in self.servo_list:
      if servo.is_open() == False:
        return False

    return True

  def activate(self):
    for activate_list in self._activate_sequence:
      for i in activate_list:
        self.servo_list[i].torque(1)
        self.servo_list[i].move(0,200)

      sleep(2.0)
 
  def deactivate(self):
    self.torque(0)

  def torque(self, mode):
    for i in range(self._num_servo):
      self.servo_list[i].torque(mode)

  def move(self, position_list, move_time = 20):
    for i in range(self._num_servo):
      self.servo_list[i].move(position_list[i], move_time)

  def get_status(self):
    _angle = []
    _time = []
    _speed = []
    _load = []
    _temperature = []
    _voltage = []
    for servo in self.servo_list:
      _data = servo.get_status()
      _angle.append(_data["Angle"])
      _time.append(_data["Time"])
      _speed.append(_data["Speed"])
      _load.append(_data["Load"])
      _temperature.append(_data["Temperature"])
      _voltage.append(_data["Voltage"])

    return {"Angle": _angle,           
            "Time": _time,
            "Speed": _speed,
            "Load": _load,
            "Temperature": _temperature,
            "Voltage": _voltage}
