import math


##### Define Data Format #####
class _Data:
    def __init__(self):
        
        self.f  = 0
        self.e  = 0
        self.rf = 0
        self.re = 0
        
        self._current_x = 0.0  
        self._current_y = 0.0
        self._current_z = 0.0

        self._desired_angle_1 = 0.0  
        self._desired_angle_2 = 0.0
        self._desired_angle_3 = 0.0

        self._MM_per_Segment = 0.0  
        self._angle_upper_bound = 0  
        self._angle_lower_bound = 0
        self._speed_upper_bound = 0.0
        self._speed_lower_bound = 0.0


#### Define an Element class #####
class _element:
    def __init__(self, x, y ,z):
        self.x = x
        self.y = y
        self.z = z
        
    def _get_Value(self, index):
        if index == 1 : return self.x
        if index == 2 : return self.y
        if index == 3 : return self.z
        
Data = _Data()        

#------------------------------------------------------------#
##### Define Kinematic Class #####    

tan30 = math.tan(math.radians(30))  
tan60 = math.tan(math.radians(60))  
cos120 = math.cos(math.radians(120)) 
sin120 = math.sin(math.radians(120))  


def _Forward_Kinematic(angle_1, angle_2, angle_3):
    t = (Data.f - Data.e) * tan30 / 2
    dtr = math.pi / 180.0

    angle_1 *= dtr
    angle_2 *= dtr
    angle_3 *= dtr

    y1 = -(t + Data.rf * math.cos(angle_1))
    z1 = -Data.rf * math.sin(angle_1)

    y2 = (t + Data.rf * math.cos(angle_2)) * math.sin(math.radians(30))
    x2 = y2 * tan60
    z2 = -Data.rf * math.sin(angle_2)

    y3 = (t + Data.rf * math.cos(angle_3)) * math.sin(math.radians(30))
    x3 = -y3 * tan60
    z3 = -Data.rf * math.sin(angle_3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 ** 2 + z1 ** 2
    w2 = x2 ** 2 + y2 ** 2 + z2 ** 2
    w3 = x3 ** 2 + y3 ** 2 + z3 ** 2

    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    a = a1 ** 2 + a2 ** 2 + dnm ** 2
    b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm ** 2)
    c = (b2 - y1 * dnm) ** 2 + b1 ** 2 + dnm ** 2 * (z1 ** 2 - Data.re ** 2)

    d = b ** 2 - 4.0 * a * c
    if d < 0:
        return None  # Indicate that there's no valid point

    z = -0.5 * ((b + math.sqrt(d)) / a)
    x = (a1 * z + b1) / dnm
    y = (a2 * z + b2) / dnm

    return x, y, z


def _Auxiliary_Function(x, y, z):
    y1 = -0.5 * 0.57735 * Data.f
    y -=  0.5 * 0.57735 * Data.e

    a = (x ** 2 + y ** 2 + z ** 2 + Data.rf ** 2 - Data.re ** 2 - y1 ** 2) / (2 * z)
    b = (y1 - y) / z

    d = -(a + b * y1) * (a + b * y1) + Data.rf * (b ** 2 * Data.rf + Data.rf)
    if d < 0:
        return None  # Indicate that there's no valid angle

    yj = (y1 - a * b - math.sqrt(d)) / (b ** 2 + 1)
    zj = a + b * yj
    angle = 180.0 * math.atan(-zj / (y1 - yj)) / math.pi + (180.0 if yj > y1 else 0.0)

    return angle


def _Inverse_Kinematic(x, y, z):
    angle_1 = _Auxiliary_Function(x, y, z)
    angle_2 = _Auxiliary_Function(x * cos120 + y * sin120,
                                        y * cos120 - x * sin120,
                                        z)
    angle_3 = _Auxiliary_Function(x * cos120 - y * sin120,
                                        y * cos120 + x * sin120,
                                        z)

    return angle_1, angle_2, angle_3


def _Check_Point_Validation(angle_1, angle_2, angle_3):
    return (Data._angle_lower_bound <= angle_1 <= Data._angle_upper_bound and
            Data._angle_lower_bound <= angle_2 <= Data._angle_upper_bound and
            Data._angle_lower_bound <= angle_3 <= Data._angle_upper_bound)

#------------------------------------------------------------#
##### Define Interpolation Class #####
           
    
def _Distance( _x_0, _y_0, _z_0, _x_1, _y_1, _z_1 ):
    return math.sqrt( math.pow((_x_0 - _x_1), 2) + 
                        math.pow((_y_0 - _y_1), 2) + 
                        math.pow((_z_0 - _z_1), 2) )
    

def _Linear_Interpolation(_x_1, _y_1, _z_1):
    
    if not _Check_Point_Validation(_x_1, _y_1, _z_1):
        return False
    
    _Number_of_Segment = int( _Distance(Data._current_x, Data._current_y, Data._current_z, _x_1, _y_1, _z_1) / Data._MM_per_Segment )
    
    if ( abs(_Number_of_Segment) < 3 ):
        return False
    
    for i in range(_Number_of_Segment):
        
        _buffer   = i / _Number_of_Segment
        _x_buffer = (1 - _buffer) * Data._current_x + _buffer * _x_1
        _y_buffer = (1 - _buffer) * Data._current_y + _buffer * _y_1
        _z_buffer = (1 - _buffer) * Data._current_z + _buffer * _z_1
        
        temp = _element(0, 0, 0)
        _Inverse_Kinematic(_x_buffer, _y_buffer, _z_buffer, temp._angle_1, temp._angle_2, temp._angle_3)
        
        _Segment.append(temp)            
        
    return True


def _Circle_Interpolation(i, j, _x_f, _y_f, _rotating_direction):
    
    if not (i == 0) and not (j == 0):
        return False
    
    _radius = math.sqrt( math.pow(i, 2) +
                                math.pow(j, 2) )
    
    _o_x = Data._current_x + i
    _o_y = Data._current_y + j
    
    _current_xy_angle = math.acos(-i / _radius)
    
    if j > 0:
        _current_xy_angle *= -1
        
    _desired_xy_angle = math.acos((_x_f - _o_x) / _radius)
    
    if (_y_f - _o_y <= 0):
        _desired_xy_angle *= -1
        
    _Angular_Distance = _desired_xy_angle - _current_xy_angle    
    
    if abs(_Angular_Distance) < 0.3:
        _Angular_Distance = -2 * math.pi if _rotating_direction else 2 * math.pi   
        
    _Number_of_Arc_Segment = _Angular_Distance * _radius / Data._MM_per_Segment
    _Number_of_Segment = int( _Angular_Distance / _Number_of_Arc_Segment )
    _Rad_per_Segment = Data._MM_per_Segment / _radius
    
    for i in range( _Number_of_Segment ):
        dx = (Data._current_x * _radius * math.cos(_Rad_per_Segment) - 
                Data._current_y * _radius * math.sin(_Rad_per_Segment) )
        
        dy = (Data._current_y * _radius * math.cos(_Rad_per_Segment) + 
                Data._current_x * _radius * math.sin(_Rad_per_Segment) )   
        
        temp = _element(0, 0, 0)
        _Inverse_Kinematic(dx, dy, Data._current_z, temp._angle_1, temp._angle_2, temp._angle_3)
        
        _Segment.append(temp)        
        
    return True    




#------------------------------------------------------------#
##### Define GCodeHandler Class #####    

_G_Code_Queue = []
_Segment      = []

    
def _G_Code_Queue_Update(Instruction):            
    if ( ( ( "X" in Instruction ) or ( "Y" in Instruction ) ) and
            ( ( "I" in Instruction ) or ( "J" in Instruction ) ) and
            ( (Instruction[1] == 0 ) or (Instruction[1] == 1 ) ) and
            Instruction[0] == "G" ) or ( Instruction[0] == "F" and
                                            ( "X" in Instruction ) or ( "Y" in Instruction ) or ( "Z" in Instruction) ) :
                    
        _G_Code_Queue.append(Instruction)
        
    else:
        
        return

    
            
def _G_Code_Execute(Input):
    keyvalue = [0] * 6
    
    if Input[0] == "G" :
        keyvalue[0] = 0
    else:
        keyvalue[0] = 1
    
    if not keyvalue[0] :
        keyvalue[1] = Input[1]
        
        if "X" in input:                
            keyvalue[2] = float(Input[Input.find('X') + 1 : Input.find('Y') - 1])
        else: 
            keyvalue[2] = Data._current_x    
            
        if "Y" in input:   
            keyvalue[3] = float(Input[Input.find('Y') + 1 : len(Input) - 1])
        else:
            keyvalue[3] = Data._current_y
        
        if "I" in input:  
            keyvalue[4] = float(Input[Input.find('I') + 1 : Input.find('J') - 1])
        
        if "J" in input:  
            keyvalue[5] = float(Input[Input.find('J') + 1 : Input.find('X') - 1])
            
    else:
        if "X" in input: 
            keyvalue[2] = float(Input[Input.find('X') + 1 : Input.find('Y') - 1])
        else:
            keyvalue[2] = Data._current_x 
            
        if "Y" in input: 
            keyvalue[3] = float(Input[Input.find('Y') + 1 : Input.find('Z') - 1])
        else:
            keyvalue[3] = Data._current_y
            
        if "Z" in input: 
            keyvalue[4] = float(Input[Input.find('Z') + 1 : len(Input) - 1])
        else:
            keyvalue[4] = Data._current_z
        
    return keyvalue



def _Segment_Update():
    
    if (not len(_G_Code_Queue) ):
        return 
    
    _tempInstruction = _G_Code_Queue.pop(0)
    _buffer = _G_Code_Execute(_tempInstruction)
    
    if not _buffer[0]:
        _Circle_Interpolation(_buffer[2], 
                                _buffer[3],
                                _buffer[4],
                                _buffer[5],
                                _buffer[1]) 
    else:
        _Linear_Interpolation(_buffer[2], 
                                _buffer[3],
                                _buffer[4])
    
    
    
def _File_Reading(_file_directory):
    with open(_file_directory, 'r') as file:
        for line in file:
            _temporary_line = line.strip()
            _G_Code_Queue_Update(_temporary_line)

def sum(a, b):
    return a + b
        
