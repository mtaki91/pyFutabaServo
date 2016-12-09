import serial, struct, time

class futaba_servo:
  def __init__(self, serial_port, servo_id):
    '''
    serial_port = serial.Serial(
    port = (e.g., '/dev/ttyUSB0')
    baudrate = (default: 115200)
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    timeout = 1
    ) 
    '''
    self._serial = serial_port
    self._id = servo_id

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
      self._serial.write(struct.pack('B', tmp))
      time.sleep(0.001)
   
  def move(self, Position, Time):
    data = [Position & 0x00FF, (Position & 0xFF00) >> 8 , Time & 0x00FF, (Time & 0xFF00) >> 8 ]
    self._send_command(0x00, 0x1E, 0x04, 0x01, data)

  
  def torque(self, Mode):
    """
    Set Servo's torque
    Mode = 0 : Off
         = 1 : On
         = 2 : Brake
    """

    data = Mode & 0x00FF
    self._send_command(0x00, 0x24, 0x01, 0x01, data)


  def get_servo_status(self):
    self._send_command(0x09, 0x00, 0x00, 0x01)

    readbuf = []
    for x in range(26):
      readbuf.append(self._serial.read())

    angle = struct.unpack('h', readbuf[7] + readbuf[8])[0]
    time = struct.unpack('h', readbuf[9] + readbuf[10])[0]
    speed = struct.unpack('h', readbuf[11] + readbuf[12])[0]
    load = struct.unpack('h', readbuf[13] + readbuf[14])[0]
    temperature = struct.unpack('h', readbuf[15] + readbuf[16])[0]
    voltage = struct.unpack('h', readbuf[17] + readbuf[18])[0]

    return {"Angle": angle,           
            "Time": time,
            "Speed": speed,
            "Load": load,
            "Temperature": temperature,
            "Voltage": voltage}
