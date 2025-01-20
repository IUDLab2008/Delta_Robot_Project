#include "Kinematic.h"

bool Kinematic::checkAngleValidation(float angle1, float angle2, float angle3)
{
    return ((angle1 <= ANGLE_UPPER_BOUND && angle1 >= ANGLE_LOWER_BOUND) &&
            (angle2 <= ANGLE_UPPER_BOUND && angle2 >= ANGLE_LOWER_BOUND) &&
            (angle3 <= ANGLE_UPPER_BOUND && angle3 >= ANGLE_LOWER_BOUND));
}

bool Kinematic::checkPointValidation(float x, float y, float z)
{
    return ((abs(x) <= X_WORKSPACE) && 
            (abs(y) <= Y_WORKSPACE) &&
            (abs(z) <= Z_WORKSPACE));
}

void Kinematic::forwardKinematic(float angle1, float angle2, float angle3, float& x, float& y, float& z)
{
    angle1 *= M_PI;
    angle2 *= M_PI;
    angle3 *= M_PI;

    float t = (L_F - L_E) * tan30/2;
    float dtr = pi/(float)180.0;

    angle1 *= dtr;
    angle2 *= dtr;
    angle3 *= dtr;

    float y1 = -(t + R_F * cos(angle1));
    float z1 = -R_F * sin(angle1);

    float y2 = (t + R_F * cos(angle2)) * sin30;
    float x2 = y2 * tan60;
    float z2 = -R_F * sin(angle2);

    float y3 = (t + R_F * cos(angle3)) * sin30;
    float x3 = -y3 * tan60;
    float z3 = -R_F * sin(angle3);

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
    float c = (b2 - y1 * dnm)*(b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - R_E * R_E);

    
    float d = b * b - (float)4.0 * a * c;
    if (d < 0) return; 

    z = -(float)0.5 * (b + sqrt(d))/a;
    x = (a1 * z + b1)/dnm;
    y = (a2 * z + b2)/dnm;

    return;
}

void Kinematic::auxiliaryFunction(float x, float y, float z, float &angle)
{
    float y1 = -0.5 * 0.57735 * L_F; 
    y -= 0.5 * 0.57735 * L_E;    

    float a = (x * x + y * y + z * z + R_F * R_F - R_E * R_E - y1 * y1)/(2 * z);
    float b = (y1 - y)/z;

    float d = -(a + b * y1)*(a + b * y1) + R_F * (b * b * R_F + R_F); 
    if (d < 0) return; 

    float yj = (y1 - a * b - sqrt(d))/(b * b + 1); 
    float zj = a + b * yj;
    angle = 180.0*atan(-zj / (y1 - yj))/pi + ((yj > y1) ? 180.0 : 0.0);

    return;
}

void Kinematic::inverseKinematic(float& angle1, float& angle2, float& angle3, float x, float y, float z)
{
    this-> auxiliaryFunction(x, y, z, angle1);
    this-> auxiliaryFunction(x * cos120 + y * sin120, y * cos120 - x * sin120, z, angle2);  
    this-> auxiliaryFunction(x * cos120 - y * sin120, y * cos120 + x * sin120, z, angle3);  

    return;
}

void Kinematic::calculateSubVelocities(float absoluteVelocity, float x0, float y0, float z0, float x1, float y1, float z1, float& xDot, float& yDot, float& zDot)
{
    xDot = (x0 - x1) / absoluteVelocity;
    yDot = (y0 - y1) / absoluteVelocity;
    zDot = (z0 - z1) / absoluteVelocity;
}

element Kinematic::calculateAngularVelocities(float x, float y, float z, float xDot, float yDot, float zDot)
{
    float b11 = L_F * ((y + aTerm) * sin(atan2(z, y + aTerm)) - z * cos(atan2(z, y + aTerm)));
    float b22 = -L_F * (sqrt3 * (x + bTerm) + (y + cTerm)) * sin(atan2(z, y + cTerm)) + 2.0 * z * cos(atan2(z, y + cTerm));
    float b33 = L_F * (sqrt3 * (x - bTerm) - (y - cTerm)) * sin(atan2(z, y - cTerm)) - 2.0 * z * cos(atan2(z, y - cTerm));

    float rpm_1 = (2 * (y + aTerm) + L_F * cos(atan2(z, y + aTerm)) * xDot + z * sin(atan2(z, y + aTerm)) * zDot) * M_PI / b11;
    float rpm_2 = (2 * (y + cTerm) - L_F * cos(atan2(z, y + cTerm)) * xDot + z * sin(atan2(z, y + cTerm)) * zDot) * M_PI / b22;
    float rpm_3 = (2 * (y - cTerm) + L_F * cos(atan2(z, y - cTerm)) * xDot + z * sin(atan2(z, y - cTerm)) * zDot) * M_PI / b33;

    return {rpm_1, rpm_2, rpm_3};

}