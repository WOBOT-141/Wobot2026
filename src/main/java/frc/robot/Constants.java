// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public final class CANIdConstants {
     public static final int frontLeftSteerMotorID = 1;
    public static final int frontLeftDriveMotorID = 2;
    public static final int backLeftSteerMotorID = 3;
    public static final int backLeftDriveMotorID = 4;
    public static final int frontRightSteerMotorID = 5;
    public static final int frontRightDriveMotorID = 6;
    public static final int backRightSteerMotorID = 7;
    public static final int backRightDriveMotorID = 8;
    public static final int frontLeftEncoderid = 9; 
    public static final int backLeftEncoderid = 10; 
    public static final int frontRightEncoderid = 11; 
    public static final int backRightEncoderid = 12;
    public static final int flyWheelRightMotorID = 13;
    public static final int flyWheelMiddlehtMotorID = 14;
    public static final int flyWheelLeftMotorID = 15;
    public static final int hoodMotorID = 20;
  }

  public static class RotateCommandConstants {
    public static final double P = 0.00025;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 3.0; // 2.0
  }

  public static class RotateVisionConstants {
    public static final double P = 0.004;
    public static final double I = 0.002;// .00001
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 2; // 2.0
  }

  public static class DriveCommandConstants {

    public static final double P = 0.024;// .0024
    public static final double I = 0.0008;
    public static final double D = 0;

    public static final double P2 = .04;
    public static final double I2 = .00012;
  }

  public static class DriveConstants {
    // wheel diameter multiplied by pi = circumference
    public static final double wheelCircumference_in = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;
    
    public static final double rotationsPerInch_rot = driveGearRatio / wheelCircumference_in; // 3.6 might be the gear ratio
    public static final double drivekP = 0.0016;// .00009
    public static final double drivekI = 0.000068;// 0
    public static final double drivekD = 0;
    public static final double drivekIz = 0;
    public static final double drivekFF = 0.00016500000492669642;
    public static final double topSpeed_mps = 25;// 20.36

    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    public static final int driveCurrentLim_amp = 80;

    public static final double driveTargetTolerance = 2.0;
    public static final double visionDrivekP = 0.0032;
    public static final double visionDrivekI = 0.00028;
    public static final double visionDrivekD = 0.00015;
  }

  public static class SteerConstants {
    public static final double steerGearRatio = 12.8;
    public static final int neoTicksPerRotation = 42;
    public static final double steerDegPerMotorRotation_deg = 360 / steerGearRatio;
    public static final double steerDegPerEncoderTick_deg = steerDegPerMotorRotation_deg / neoTicksPerRotation;
    public static final double kF = 0;
    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int steerContCurrentLim_amp = 4;
    public static final int steerPeakCurrentLim_amp = 6;
  }
  
  public static class ControllerOneConstants {
    /**
     * Factors used in the spektrum flight controller to multiply the raw stick
     * outputs in order to scale them up or down.
     * These can be found by dividing 1 by the raw stick output of that direction
     */
    public static final double RotateFactor = 1.35;
    public static final double DriveXFactor = 1.26;
    public static final double DriveYFactor = 1.23;
    public static final double ThrottleFactor = 1.55;
    /**
     * Offsets used to add or subtract from a raw stick value in the spektrum flight
     * controller to make their resting postion 0
     * These should be equal to the raw stick output at rest.
     */
    public static final double RotateOffset = -0.056375;
    public static final double DriveXOffset = 0.039816 + 0.029202; // .075813
    public static final double DriveYOffset = -0.024228;
    public static final double ThrottleOffset = 0;
  }
  
   public static class ShooterConstants {
    public static final double wheelCircumference_in = (Math.PI * 4); 
    public static final double diameter_in = 4;
    public static final double flywheelGearRatio = 29.0/24.0; // output 29:24 input 
    
    public static final double P = 0.0016;// .00009
    public static final double I = 0.000068;// 0
    public static final double D = 0;
    public static final double FF = 0.00016500000492669642;

    public static final int flywheelTolernce_ips = 13;
    public static final int slewRateLimit = 4000;
    public static final int stallLimit_amp = 60;

    public static final double hoodP = 0;//TODO tune
    public static final double hoodI = 0;
    public static final double hoodD = 0;
    public static final double hoodFF = 0.00016500000492669642;

    public static final double hoodHomeRPM = 0;
    public static final double hoodHomeCurrentLim_amp = 20;
    public static final double steerDegPerEncoderTick_deg = 5; //5 motor ticks = 1 degree max of 49 and min of 89 degreseor or  visas versa 
    public static final double hoodRevsPerDegree = (62/10) * (319/50); //10:62 & 50:319


  }
}
