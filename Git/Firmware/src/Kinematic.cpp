#include "Kinematic.h"


// Forward Kinematic Function
void _Kinematic::_Forward_Kinematic(float _angle_1, float _angle_2, float _angle_3, float &x, float &y, float &z)
{
    float t = (f - e) * tan30/2;
    float dtr = pi/(float)180.0;

    _angle_1 *= dtr;
    _angle_2 *= dtr;
    _angle_3 *= dtr;

    float y1 = -(t + rf * cos(_angle_1));
    float z1 = -rf * sin(_angle_1);

    float y2 = (t + rf * cos(_angle_2)) * sin30;
    float x2 = y2 * tan60;
    float z2 = -rf * sin(_angle_2);

    float y3 = (t + rf * cos(_angle_3)) * sin30;
    float x3 = -y3 * tan60;
    float z3 = -rf * sin(_angle_3);

    float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    float w1 = y1 * y1 + z1 * z1;
    float w2 = x2 * x2 + y2 * y2 + z2 * z2;
    float w3 = x3 * x3 + y3 * y3 + z3 * z3;

    
    float a1 =   (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1))/2.0;

    
    float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2)/2.0;

    
    float a = a1 * a1 + a2 * a2 + dnm * dnm;
    float b = 2*(a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    float c = (b2 - y1 * dnm)*(b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

    
    float d = b * b - (float)4.0 * a * c;
    if (d < 0) return; 

    z = -(float)0.5 * (b + sqrt(d))/a;
    x = (a1 * z + b1)/dnm;
    y = (a2 * z + b2)/dnm;

    return;
}


void _Kinematic::_Auxiliary_Function(float x, float y, float z, float &_angle)
{
    float y1 = -0.5 * 0.57735 * f; 
    y -= 0.5 * 0.57735 * e;    

    float a = (x * x + y * y + z * z + rf * rf - re * re - y1 * y1)/(2 * z);
    float b = (y1 - y)/z;

    float d = -(a + b * y1)*(a + b * y1) + rf*(b * b * rf + rf); 
    if (d < 0) return; 

    float yj = (y1 - a * b - sqrt(d))/(b * b + 1); 
    float zj = a + b * yj;
    _angle = 180.0*atan(-zj / (y1 - yj))/pi + ((yj > y1) ? 180.0 : 0.0);

    return;
}


void _Kinematic::_Inverse_Kinematic(float &_angle_1, float &_angle_2, float &_angle_3, float x, float y, float z)
{
    this->_Auxiliary_Function(x, y, z, _angle_1);
    this->_Auxiliary_Function(x * cos120 + y * sin120, y * cos120 - x * sin120, z, _angle_2);  
    this->_Auxiliary_Function(x * cos120 - y * sin120, y * cos120 + x * sin120, z, _angle_3);  

    return;
}


bool _Kinematic::_Check_Point_Validation(float _angle_1, float _angle_2, float _angle_3)
{

    return ( (_angle_1 <= Data._angle_upper_bound) && (_angle_1 >= Data._angle_lower_bound) &&
         (_angle_2 <= Data._angle_upper_bound) && (_angle_2 >= Data._angle_lower_bound) &&
         (_angle_3 <= Data._angle_upper_bound) && (_angle_3 >= Data._angle_lower_bound) );

}

