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

    # old pyserial has "readall" instead of "read_all"
    if "readall" in dir(self):
      self.read_all = self.readall
      
  def connect(self):
    self.isBusy = False
    if self.isOpen == True:
      return True

    else:  
      try:
        self.open()
        return True
    
      except:
        return False


class futaba_servo(object):
  def __init__(self, serial_conv, servo_id, zero_position = 0):
    self._serial_conv = serial_conv
    self._id = servo_id
    self._zero_pos = zero_position

  def is_open(self):
    return self._serial_conv.isOpen()

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
    send_data = ""
    for tmp in command:
      send_data = send_data + struct.pack("B", tmp)

#    print send_data
    self._serial_conv.write(send_data)
    sleep(0.005)
    
#    for tmp in command:
#      self._serial_conv.write(struct.pack('B', tmp))
#      sleep(0.001)

  def fix_connection(self):
    self._serial_conv.isBusy = True
    while True:
      self._send_command(0x01, 0x00, 0x00, 0x01)
      sleep(0.015)
      if self._serial_conv.inWaiting() > 1:
        c = self._serial_conv.read_all()
      elif self._serial_conv.inWaiting() == 1:
        c = self._serial_conv.read()
      else: # len(c) == 0:
	print "retry to connect ",self._serial_conv.port, " ID: ", self._id
        continue
      if c[-1] == '\x07':
        break
      sleep(0.01)
    self._serial_conv.isBusy = False
      

  def set_compliance_param(self, CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch):
    data = [CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch & 0x00FF, (Punch & 0xFF00) >> 8]

    while self._serial_conv.isBusy:
      sleep(0.001)

    self._serial_conv.isBusy = True
    self._send_command(0x00, 0x18, 0x06, 0x01, data)
    self._serial_conv.isBusy = False


  def move(self, Position, Time):
    Position = int(Position + self._zero_pos)
    Time = int(Time)
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
    
    if self._serial_conv.inWaiting() > 0:
      self._serial_conv.read_all()
      
    self._send_command(0x09, 0x00, 0x00, 0x01)
    readbuf = []
    for x in range(26):
      char = self._serial_conv.read()
      readbuf.append(char)

    self._serial_conv.isBusy = False
    
    _angle = struct.unpack('h', readbuf[7] + readbuf[8])[0]
    _time = struct.unpack('h', readbuf[9] + readbuf[10])[0]
    _speed = struct.unpack('h', readbuf[11] + readbuf[12])[0]
    _load = struct.unpack('h', readbuf[13] + readbuf[14])[0]
    _temperature = struct.unpack('h', readbuf[15] + readbuf[16])[0]
    _voltage = struct.unpack('h', readbuf[17] + readbuf[18])[0]

    return {"Angle": _angle - self._zero_pos,           
            "Time": _time,
            "Speed": _speed,
            "Load": _load,
            "Temperature": _temperature,
            "Voltage": _voltage}


class servo_cluster(object):
  def __init__(self, servo_list, activate_sequence = [], zero_position = []):
    self.servo_list = servo_list
    self._num_servo = len(servo_list)

    if zero_position == []:
      zero_position = [0 for i in range(self._num_servo)]

    self.set_zero_position(zero_position)
    
    if activate_sequence == []:
      activate_sequence = [range(self._num_servo)]
    
    self._activate_sequence = activate_sequence

  def is_open(self):
    for servo in self.servo_list:
      if servo.is_open() == False:
        return False

    return True

  def fix_connection(self):
    for servo in self.servo_list:
      servo.fix_connection()
    sleep(0.01)
    
  def set_zero_position(self, angle_list):
    for i in range(self._num_servo):
      self.servo_list[i]._zero_pos = angle_list[i]

  def activate(self):
    for activate_list in self._activate_sequence:
      for i in activate_list:
        self.servo_list[i].torque(1)
        self.servo_list[i].move(0,200)

      sleep(2.0)
 
  def deactivate(self):
    self.torque(0)

  def torque(self, mode):
    self.fix_connection()
    if isinstance(mode, int):
      for servo in self.servo_list:
        servo.torque(mode)

    if isinstance(mode, list):
      for i in range(self._num_servo):
        self.servo_list[i].torque(mode[i])


  def set_compliance_param(self, servo_index, CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch):
    self.fix_connection()
    if isinstance(servo_index, int):
      self.servo_list[servo_index].set_compliance_param(CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)

    if isinstance(servo_index, list):
      for i in servo_index:
        self.servo_list[i].set_compliance_param(CW_Margin, CCW_Margin, CW_Slope, CCW_Slope, Punch)

  def move(self, position_list, move_time = 20, servo_index = [], reverse = []):
    self.fix_connection()
    if servo_index == []:
      servo_index = range(self._num_servo)
    if reverse == []:
      reverse = [False for i in range(self._num_servo)]

    for i,j in enumerate(servo_index):
      angle = position_list[i]
      if angle is None:
        continue
      if reverse[i]:
        angle *= -1
      self.servo_list[j].move(angle, move_time)

  def get_status(self):
    self.fix_connection()
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

