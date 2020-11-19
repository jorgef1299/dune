//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Jorge Ferreira                                                    *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 0

#define G_FORCE 9.800054

namespace Sensors
{
  namespace MPU9250
  {
    using DUNE_NAMESPACES;
    
    //! Task arguments.
    struct Arguments
    {
      //! I2C device.
      std::string i2c_dev;
      //! Gyroscope offset bias correction value.
      std::vector<int16_t> gyroscope_offset;
      //! Accelerometer offset bias correction value.
      std::vector<float> accel_offset;
      //! Accelerometer scale correction value.
      std::vector<float> accel_scale;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Angular velocity.
      IMC::AngularVelocity m_ang_vel;
      //! Acceleration
      IMC::Acceleration m_accel;
      //! Magnetic field.
      IMC::MagneticField m_magn;
      //! Euler angles.
      IMC::EulerAngles m_euler;
      //! I2C handle.
      I2C* m_i2c;
      //! Device I2C address.
      const uint8_t dev_addr = 0x68;
      //! Device registers addresses.
      const uint8_t WHO_AM_I = 0x75;
      const uint8_t PWR_MGMT_1 = 0x6B;
      const uint8_t ACCEL_CONFIG = 0x1C;
      const uint8_t GYRO_CONFIG = 0x1B;
      const uint8_t ACCEL_XOUT_H = 0x3B;
      const uint8_t ACCEL_YOUT_H = 0x3D;
      const uint8_t ACCEL_ZOUT_H = 0x3F;
      const uint8_t GYRO_XOUT_H = 0x43;
      const uint8_t GYRO_YOUT_H = 0x45;
      const uint8_t GYRO_ZOUT_H = 0x47;
      
      float declination = 97;
      float beta = 0.1f;
      float q0 = 1.0f;
      float q1 = 0.0f;
      float q2 = 0.0f;
      float q3 = 0.0f;
      
      double lastUpdate = Time::Clock::getSinceEpoch();
    
      //! Task arguments.
      Arguments m_args;
      
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        // Define configuration parameters.
        param("I2C - Device", m_args.i2c_dev)
        .defaultValue("")
        .description("I2C Device");
        
        param("Gyroscope Offset", m_args.gyroscope_offset)
        .defaultValue("-768, 0, 196")
        .size(3)
        .description("Gyroscope offset correction values");
        
        param("Accelerometer Offset", m_args.accel_offset)
        .defaultValue("0, 0, 0")
        .size(3)
        .description("Accelerometer offset correction values");
        
        param("Accelerometer Scale Correction", m_args.accel_scale)
        .defaultValue("1, 1, 1")
        .size(3)
        .description("Accelerometer scale correction values");
        
        bind<IMC::MagneticField>(this);
      }
      
      //! Consume magnetic field message.
      void consume(const IMC::MagneticField* msg)
      {
        for (int i = 0; i < 10; i++) 
          MadgwickUpdate(m_ang_vel.x, -m_ang_vel.y, -m_ang_vel.z, -m_accel.x, m_accel.y, m_accel.z, msg->x, -msg->y, -msg->z);
	 computeEulerAngles();
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        uint8_t whoAmI_result = 0;
        // Establish an I2C connection to the device.
        m_i2c = new I2C(m_args.i2c_dev);
        m_i2c->connect(dev_addr);
        // Check to see if there is a good connection with the MPU9250.
        whoAmI_result = readByte(&WHO_AM_I);
        if(whoAmI_result == 0x71)
        {
          writeByte(PWR_MGMT_1, 0x00); // Activate / reset the IMU.
          writeByte(ACCEL_CONFIG, 0x00); // 2G Full scale.
          writeByte(GYRO_CONFIG, 0x00); // 250dps Full scale.
        }
        else
          throw std::runtime_error("IMU WHO_AM_I is wrong.");
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        if(CALIBRATE_ACCEL)
          calibrateAccel(200);
        if(CALIBRATE_GYRO)
          calibrateGyro(200);
      }

      //! Send data using the I2C protocol.
      void
      writeByte(uint8_t registerAddress, uint8_t value)
      {
        uint8_t data[2];
      
        data[0] = registerAddress;
        data[1] = value;
        m_i2c->write(data, 2);
      }
      
      //! Read data using the I2C protocol.
      uint8_t
      readByte(const uint8_t* registerAddress)
      {
        uint8_t value = 0;
        m_i2c->write(registerAddress, 1);
        m_i2c->read(&value, 1);
        return value;
      }
      
      //! Read a two bytes value, stored as LSB and MSB, in 2's complement.
      int16_t
      readWord(const uint8_t* register_addr_high)
      {
        uint8_t highByte = readByte(register_addr_high);
        uint8_t lowByte = readByte(register_addr_high + 1);
        uint16_t word = (highByte << 8) + lowByte;
        
        if(word >= 32768)
          return (word - 65536);
        
        return word;
      } 
      
      //! Print only the data from one specific axis.
      int16_t
      readOneAccelAxis(uint8_t numberOfReadings, char axis)
      {
        int32_t calibrationValue = 0;
        int16_t *p;
        uint8_t offs = 0;
        
        switch(axis)
        {
          case 'x': 
            offs = 0;
            break;
          case 'y':
            offs = 1;
            break;
          case 'z':
            offs = 2;
            break;
          default:
            offs = 0;
        }
        for(int i = 0; i < numberOfReadings; i++)
        {
          p = readRawAccel();
          inf("Eixo: %c\tValor: %d", axis, *(p+offs));
          calibrationValue += *(p + offs);
          Time::Delay::waitMsec(5);
        }
        return calibrationValue / numberOfReadings;
      }
      
      //! Calibrate the accelerometer.
      void
      calibrateAccel(uint8_t numberOfReadings)
      {
        int16_t zMax;
        int16_t zMin;
        int16_t yMax;
        int16_t yMin;
        int16_t xMax;
        int16_t xMin;
        
        inf("Calibrating accelerometer with %d points.", numberOfReadings);
        inf("Place it on the position Z+");
        Time::Delay::wait(10);
        zMax = readOneAccelAxis(numberOfReadings, 'z');
        inf("Place it on the position Z-");
        Time::Delay::wait(10);
        zMin = readOneAccelAxis(numberOfReadings, 'z');
        inf("Place it on the position Y+");
        Time::Delay::wait(10);
        yMax = readOneAccelAxis(numberOfReadings, 'y');
        inf("Place it on the position Y-");
        Time::Delay::wait(10);
        yMin = readOneAccelAxis(numberOfReadings, 'y');
        inf("Place it on the position X+");
        Time::Delay::wait(10);
        xMax = readOneAccelAxis(numberOfReadings, 'x');
        inf("Place it on the position X-");
        Time::Delay::wait(10);
        xMin = readOneAccelAxis(numberOfReadings, 'x');
        // Calculate offset bias and scale correction.
        m_args.accel_offset[0] = (float) (xMin + xMax) / 2;
        m_args.accel_offset[1] = (float) (yMin + yMax) / 2;
        m_args.accel_offset[2] = (float) (zMin + zMax) / 2;
        m_args.accel_scale[0] = (float) 16384 / ((abs(xMin) + abs(xMax)) / 2);
        m_args.accel_scale[1] = (float) 16384 / ((abs(yMin) + abs(yMax)) / 2);
        m_args.accel_scale[2] = (float) 16384 / ((abs(zMin) + abs(zMax)) / 2);
        
        inf("Calibration completed!");
        inf("X axis offset: %f\tY axis offset: %f\tZ axis offset: %f", m_args.accel_offset[0], m_args.accel_offset[1], m_args.accel_offset[2]);
        inf("X axis scale: %f\tY axis scale: %f\tZ axis scale: %f", m_args.accel_scale[0], m_args.accel_scale[1], m_args.accel_scale[2]);
        Time::Delay::wait(300); // Give some time to take note of this values
      }
      
      //! Calibrate the gyroscope
      void
      calibrateGyro(uint8_t numberOfReadings)
      {
        int16_t *p;
        float gyroOffset[3] = {0, 0, 0};
        
        inf("Calibrating gyro with %d points. Please do not move the vehicle.", numberOfReadings);
        for(uint8_t i = 0; i < numberOfReadings; i++)
        {
          p = readRawGyro();
          gyroOffset[0] += (*p);
          gyroOffset[1] += *(p + 1);
          gyroOffset[2] += *(p + 2);
        }
        gyroOffset[0] /= numberOfReadings;
        gyroOffset[1] /= numberOfReadings;
        gyroOffset[2] /= numberOfReadings;
        
        inf("Calibration completed!");
        inf("X axis offset: %f\tY axis offset: %f\tZ axis offset: %f", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
        for(uint8_t i = 0; i < 3; i++)
          m_args.gyroscope_offset[i] = gyroOffset[i];
        Time::Delay::wait(30); // Give some time to take note of this values
      }
      
      //! Read raw data from the accelerometer.
      int16_t*
      readRawAccel()
      {
        static int16_t accelRaw[3];
        
        accelRaw[0] = readWord(&ACCEL_XOUT_H);
        accelRaw[1] = readWord(&ACCEL_YOUT_H);
        accelRaw[2] = readWord(&ACCEL_ZOUT_H);
        
        return accelRaw;
      }
      
      //! Read raw data from the gyroscope.
      int16_t*
      readRawGyro()
      {
        static int16_t gyroRaw[3];
        
        gyroRaw[0] = readWord(&GYRO_XOUT_H);
        gyroRaw[1] = readWord(&GYRO_YOUT_H);
        gyroRaw[2] = readWord(&GYRO_ZOUT_H);
        
        return gyroRaw;
      }
      
      //! Read raw data from the accelerometer and do some corrections
      void 
      readAccel()
      {
        int16_t *p;
        double imc_tstamp = Clock::getSinceEpoch();
        
        p = readRawAccel();
        // Convert to g force.
        m_accel.x = (((*p) - m_args.accel_offset[0]) * m_args.accel_scale[0]) / 16384;
        m_accel.y = ((*(p + 1) - m_args.accel_offset[1]) * m_args.accel_scale[1]) / 16384;
        m_accel.z = ((*(p + 2) - m_args.accel_offset[2]) * m_args.accel_scale[2]) / 16384;
        // Convert to m/s/s
        m_accel.x *= G_FORCE;
        m_accel.y *= G_FORCE;
        m_accel.z *= G_FORCE;
        m_accel.setTimeStamp(imc_tstamp);
        //inf("%f\t%f\t%f", m_accel.x, m_accel.y, m_accel.z);
        dispatch(m_accel, DF_KEEP_TIME);
      }
      
      //! Read raw data from the gyroscope and do some corrections
      void
      readGyro()
      {
        int16_t *p;
        double imc_tstamp = Clock::getSinceEpoch();
        
        p = readRawGyro();
        m_ang_vel.x = ((*p) - m_args.gyroscope_offset[0]) / 131.072;
        m_ang_vel.y = (*(p + 1) - m_args.gyroscope_offset[1]) / 131.072;
        m_ang_vel.z = (*(p + 2) - m_args.gyroscope_offset[2]) / 131.072;
        
        // Convert to rad/s
        m_ang_vel.x = Angles::radians(m_ang_vel.x);
        m_ang_vel.y = Angles::radians(m_ang_vel.y);
        m_ang_vel.z = Angles::radians(m_ang_vel.z);
        m_ang_vel.setTimeStamp(imc_tstamp);
        dispatch(m_ang_vel, DF_KEEP_TIME);
	//inf("%f\t%f\t%f", m_ang_vel.x, m_ang_vel.y, m_ang_vel.z);
      }
      
      //! Apply the madwick filter (sensor fusion data)
      void
      MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
      {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        
        double now = Time::Clock::getSinceEpoch();
        double deltaT = now - lastUpdate;
        lastUpdate = now;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

          // Normalise accelerometer measurement
          recipNorm = invSqrt(ax * ax + ay * ay + az * az);
          ax *= recipNorm;
          ay *= recipNorm;
          az *= recipNorm;

          // Normalise magnetometer measurement
          recipNorm = invSqrt(mx * mx + my * my + mz * mz);
          mx *= recipNorm;
          my *= recipNorm;
          mz *= recipNorm;

          // Auxiliary variables to avoid repeated arithmetic
          _2q0mx = 2.0f * q0 * mx;
          _2q0my = 2.0f * q0 * my;
          _2q0mz = 2.0f * q0 * mz;
          _2q1mx = 2.0f * q1 * mx;
          _2q0 = 2.0f * q0;
          _2q1 = 2.0f * q1;
          _2q2 = 2.0f * q2;
          _2q3 = 2.0f * q3;
          q0q0 = q0 * q0;
          q0q1 = q0 * q1;
          q0q2 = q0 * q2;
          q0q3 = q0 * q3;
          q1q1 = q1 * q1;
          q1q2 = q1 * q2;
          q1q3 = q1 * q3;
          q2q2 = q2 * q2;
          q2q3 = q2 * q3;
          q3q3 = q3 * q3;
        
          // Reference direction of Earth's magnetic field
          hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
          hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
          _2bx = sqrt(hx * hx + hy * hy);
          _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
          _4bx = 2.0f * _2bx;
          _4bz = 2.0f * _2bz;
          _8bx = 2.0f * _4bx;
          _8bz = 2.0f * _4bz;

          // Gradient decent algorithm corrective step
          s0= -_2q2*(2.0f*(q1q3 - q0q2) - ax)    +   _2q1*(2.0f*(q0q1 + q2q3) - ay)   +  -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
          s1= _2q3*(2.0f*(q1q3 - q0q2) - ax) +   _2q0*(2.0f*(q0q1 + q2q3) - ay) +   -4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
          s2= -_2q0*(2.0f*(q1q3 - q0q2) - ax)    +     _2q3*(2.0f*(q0q1 + q2q3) - ay)   +   (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
          s3= _2q1*(2.0f*(q1q3 - q0q2) - ax) +   _2q2*(2.0f*(q0q1 + q2q3) - ay)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
          recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
          s0 *= recipNorm;
          s1 *= recipNorm;
          s2 *= recipNorm;
          s3 *= recipNorm;

          // Apply feedback step
          qDot1 -= beta * s0;
          qDot2 -= beta * s1;
          qDot3 -= beta * s2;
          qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * deltaT;
        q1 += qDot2 * deltaT;
        q2 += qDot3 * deltaT;
        q3 += qDot4 * deltaT;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
      }
      
      float invSqrt(float x)
      {
        float tmp = 1/(sqrt(x));
        return tmp;
      }

      //! Get Euler Angles and dispatch them.
      void computeEulerAngles()
      {
	double imc_tstamp = Clock::getSinceEpoch();
        m_euler.phi = Angles::normalizeRadian(atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2));
        m_euler.theta = Angles::normalizeRadian(asinf(-2.0f * (q1*q3 - q0*q2)));
        m_euler.psi_magnetic = Angles::normalizeRadian(atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3));
        m_euler.psi = Angles::normalizeRadian(m_euler.psi_magnetic + Angles::radians(declination));
        m_euler.setTimeStamp(imc_tstamp);
	dispatch(m_euler, DF_KEEP_TIME);
	//inf("%.2f %.2f %.2f", Angles::degrees(m_euler.phi), Angles::degrees(m_euler.theta), Angles::degrees(m_euler.psi_magnetic));
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
	  consumeMessages();
          readGyro();
          readAccel();
        }
      }
    };
  }
}

DUNE_TASK

