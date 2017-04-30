#include <FaBo9Axis_MPU9250.h>

FaBo9Axis fabo_9axis;    // For MPU9250 readings
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion

float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method
    float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float magCalibration[3] = { 39.87, 40.255, 32.465};
float magBias[3] = { -18.94, 132.785, -14.315};
    
    // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
    const float Kp= 5.0f; 
    const float Ki = 0.1f;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    fabo_9axis.begin();
    magCal();
}

void loop() {
  // put your main code here, to run repeatedly:
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat; // sum for averaging filter update 

    //getYaw();
    Serial.println(getYaw());
    sum = 0; 
    delay(200);
}

float getYaw()
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float mRes = 10.*4912./32760.0;
    float gRes = 250.0/32768.0;
    float aRes = 2.0/32768.0;
    

    fabo_9axis.readAccelXYZ(&ax, &ay, &az);
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    fabo_9axis.readMagnetXYZ(&mx, &my, &mz);

//    ax*=aRes;
//    ay*=aRes;
//    az*=aRes;
//
//    gx*=gRes;
//    gy*=gRes;
//    gz*=gRes;
//
//    mx*=mRes*magCalibration[0];
//    my*=mRes*magCalibration[1];
//    mz*=mRes*magCalibration[2];

    //Original
//    mx=(mx-10.285)*42.575;
//    my=(my-59.015)*41.895;
//    mz=(mz+26.44)*35.17;

//    mx=(mx-16.42)*45.28;
//    my=(my-105.375)*30.875;
//    mz=(mz+20.69)*20.855;

    mx = (mx - magBias[0])/magCalibration[0];
    my = (my - magBias[1])/magCalibration[1];
    mz = (mz - magBias[2])/magCalibration[2];

//    Serial.print("ax: ");
//    Serial.print(ax);
//    Serial.print(" ay: ");
//    Serial.print(ay);
//    Serial.print(" az: ");
//    Serial.println(az);
//    
//    Serial.print("gx: ");
//    Serial.print(gx);
//    Serial.print(" gy: ");
//    Serial.print(gy);
//    Serial.print(" gz: ");
//    Serial.println(gz);
//    
//    Serial.print("mx: ");
//    Serial.print(mx);
//    Serial.print(" my: ");
//    Serial.print(my);
//    Serial.print(" mz: ");
//    Serial.println(mz);
//
//    Serial.print("q0 = "); Serial.print(q[0]);
//    Serial.print(" qx = "); Serial.print(q[1]); 
//    Serial.print(" qy = "); Serial.print(q[2]); 
//    Serial.print(" qz = "); Serial.println(q[3]); 

    MahonyQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f, my, mx, mz);

    float yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    yaw *= 180.0f / PI;

    return yaw;
}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

void magCal()
{
    float mx, my, mz;
    float maxX, maxY, maxZ;
    float minX, minY, minZ;

    Serial.println("Calibrating");

    for(int i=0; i<200;i++)
    {
        fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
        if(mx > maxX)
            maxX = mx;
        if(my > maxY)
            maxY = my;
        if(mz > maxZ)
            maxZ = mz;
        if(mx < minX)
            minX = mx;
        if(my < minY)
            minY = my;
        if(mz < minZ)
            minZ = mz;

        delay(200);
    }
    
    magBias[0] = (maxX + minX)/2;
    magBias[1] = (maxY + minY)/2;
    magBias[2] = (maxZ + minZ)/2;

    magCalibration[0] = (maxX - minX)/2;
    magCalibration[1] = (maxY - minY)/2;
    magCalibration[2] = (maxZ - minZ)/2;

    Serial.print("Bias"); Serial.print(magBias[0]); Serial.print(", "); Serial.print(magBias[1]); Serial.print(", "); Serial.println(magBias[2]);
    Serial.print("Scale"); Serial.print(magCalibration[0]); Serial.print(", "); Serial.print(magCalibration[1]); Serial.print(", "); Serial.println(magCalibration[2]);
}

