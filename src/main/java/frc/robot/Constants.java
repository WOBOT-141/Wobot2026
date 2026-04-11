// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.
 */
public final class Constants {

  public static final class CANIdConstants {
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
    public static final int flyWheelRightMotorID = 14;
    public static final int flyWheelMiddleMotorID = 13;
    public static final int flyWheelLeftMotorID = 15;
    public static final int feedMotorID = 16;
    public static final int magazineMotorID = 17;
    public static final int intakeElevationRightMotorID = 18;
    public static final int intakeRollerLeaderMotorID = 19;
    public static final int hoodMotorID = 20;
    public static final int intakeElevationLeftMotorID = 21;
    public static final int intakeRollerFollowerMotorID = 22;


    public static final int climberMotorID = 99; //TODO add can ids
    public static final int climbFollowerMotorID = 99;
  }


  public static class VisionServerConstants {
    public static final int NumberOfCameras = 2;
  }


  public static class ClimberConstants {
    public static final double homeRPM = 0;
    
    public static final double extendClimberToL1_IN = 0; 
    public static final double l1climb_IN = 0; 
    public static final double l2AndL3climb_IN = 0;
    public static final double retractClimberL1_IN = 0;
    public static final double retractClimberL2AndL3_IN = 0;

    public static final double gearRatio = 0; 

    public static final double motorRPM = 0;
    public static final int  motorFreeLimit = 40;
    public static final int motorStallLimit = 40;

    public static final double hookServoExtend = 0;
    public static final double hookServoRetract = 0;
    public static final double ratchetServoUnlock = 0;

    public static final int topLeftLimitSwitchID = 0;
    public static final int bottomLeftLimitSwitchID = 1;
    public static final int topRightLimitSwitchID = 2;
    public static final int bottomRightLimitSwitchID = 3;

    public static final double positionTolerance_IN = 1;
  }


  public static class ControllerConstants {
    public static final double rumbleFeedback_perc = 1;
  }


  public static class DriveCommandConstants {
    public static final double P = 0.024;
    public static final double I = 0.0008;
    public static final double D = 0;

    public static final double P2 = .04;
    public static final double I2 = .00012;
  }

  
  public static final class FeederConstants {
    public static final int feedMotorFreeLimit_amp = 60;
    public static final int magazineMotorFreeLimit_amp = 60;

    // Jam detection
    public static final double jamRPMThreshold_rpm = 200.0;
    public static final double jamTime_sec = 0.40;
    public static final double jamClearTime_sec = 0.25;

    // Feed behavior
    public static final double reverseSpeed_perc = -0.40;
    public static final double shooterNominalRPM_rpm = 5000.0;
    public static final double shotRecoveryTime_sec = 0.15;

    // Brownout protection
    public static final double BrownoutStartVoltage_V = 10.8;
    public static final double BrownoutCriticalVoltage_V = 9.0;

    // Scaling when voltage is low
    public static final double brownoutScale_perc = 0.8;
  }


  public static final class IntakeConstants {
    public static final int extensionCurrentLimit_amp = 40;
    public static final int rollerCurrentLimit_amp = 40;

    // Homing (current-based)
    public static final double homingSpeed_perc = -0.20;
    public static final double homingCurrentThreshold_amp = 40.0;
    public static final double homingTimeout_sec = 2.0;

    // Intake positions
    public static final double upAngle_deg = 0.0;
    public static final double downAngle_deg = 95.0;

    // Motion speeds
    public static final double intakeRetractSpeed_perc = -0.25; //-0.4;
    public static final double intakeExtendSpeed_perc = 0.15;//0.4;

    public static final double positionTolerance_deg = 3.0; // Allowed error band for position checks
    public static final double moveTimeout_sec = .20; // Safety timeout while moving

    public static final double twoTurnDistance =1; //Math.abs(-85408.851563-(-84688.945313));

    // Brownout protection
    public static final double brownoutStartVoltage_V = 10.8; // Voltage where we start scaling
    public static final double brownoutCriticalVoltage_V = 9.8; // Voltage where we heavily limit
    public static final double brownoutScale_perc = 0.8; // Scale factor when in brownout
    
    public static final double degPerMotorRotation_deg = 360.0; //TODO change to reflect elevation motor gear ratio
    public static final double intakeRawExtension_rot =  15.6;//15.38;

    public static final double SlowSpot = 5.0;

    public static final double intakeRollerP = 0.0;
    public static final double intakeRollerI = 0.0;
    public static final double intakeRollerD = 0.0;
    public static final double intakeRollerFF = 0.00015;

    public static final double rollerInSpeed_rpm = 0.4; //2250.0; //rpm max 6,300 //3500
    public static final double rollerInMaxSpeed_rpm = 0.8 ;
    public static final double rollerOutSpeed_rpm = -0.6; //-2250.0; //rpm max 6,300 //3500
  }

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public static class RotateCommandConstants {
    public static final double P = 0.0035;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 3.0;
  }

  public static class RotateVisionConstants {
    public static final double P = 0.006; //0.006
    public static final double I = 0.00002;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 2;
  }

  public static class ShooterConstants {

    public static final double g_inps2 = Units.metersToInches(9.807);

    public static final double flywheelDiameter_in = 4;
    public static final double flywheelCircum_in = Math.PI * flywheelDiameter_in;
    public static final double flywheelGearRatio = 29 / 24;
    public static final double flywheelVelocity_RPMtoIPS = (flywheelCircum_in * flywheelGearRatio / 60.0) / 2.0; // /2.0 accounts for balls spin
    public static final double flywheelSlipFactor = 1.7; //2.3
    
    public static final double muzzleHeight_in = 27;
    public static final double hubRimToCenter_in = 23.5;
    public static final double hubRimHeight_in = 72;
    public static final double muzzleToRimHeight_in = hubRimHeight_in - muzzleHeight_in;
    public static final double tagCenterToFloorHeight_in = 44.25;
    public static final double cameraToMuzzleDist_in = 10;

    public static final double leftP = 0.0017; //0.0015;
    public static final double middleP = 0.002;//0.0025;
    public static final double rightP = 0.002; //0.0012;
    public static final double I = 0.0000;
    public static final double D = 0.000000;
    public static final double kF = 0.0018; //0.0018;

    public static final double hoodP = 0.08;
    public static final double hoodI = 0.00005;
    public static final double hoodD = 0; //.001
    public static final double hood_kF = 0;
    
    public static final double hoodGearRatio = (10.0 / 62.0) * (50.0 / 319.0);
    public static final double hoodDegPerMotorRot = 360 * hoodGearRatio;
    
    // Control tuning
    public static final double flywheelVelocityTolerance_ips = 15; //25 //15
    public static final double hoodPositionTolerance_deg = 5;  //2
    public static final double hoodSlewRate = 0.8;

    // Current limits
    public static final int flywheelCurrentLimit_amp = 40;
    public static final int hoodCurrentLim_amp = 40;//20;

    // Brownout protection
    public static final double BrownoutStartVoltage_V = 10.8;
    public static final double BrownoutCriticalVoltage_V = 9.0;

    // Shot detection via RPM dip
    public static final double shotDipThreshold_rpm = 250.0;
    public static final double shotMinSeparation_sec = 0.08;
  
    public static final double hoodHomeRPM = 0.2;
    
    public static final double hoodUpperLimit_deg = 89;
    public static final double hoodLowerLimit_deg = 46;

    // Fire command constants
    public static final double firingWaitTime_sec = 0.5;
    public static final double defaultHeightAboveRim_in = 36;
  }

  public static class WheelConstants {
    public static final double steerGearRatio = 12.8;
    public static final double steerDegPerMotorRotation_deg = 360 / steerGearRatio;
    
    public static final double topSpeed_fps = 21; // TODO verify this

    public static final int driveCurrentLimit_amp = 50;
    public static final int steerCurrentLimit_amp = 50;

    public static final double DriveP = 0.0000075; // .0150
    public static final double DriveI = 0;
    public static final double DriveD = 0;
    public static final double DriveIz = 0;
    public static final double DriveFFkS = 0.00012; // Ks = min volts to make motor barely move
    public static final double DriveFFKv = 0.473; // Kv = got number from neo documentation

    public static final double SteerP = 0.025;
    public static final double SteerI = 0;
    public static final double SteerD = 0;
    public static final double SteerIz = 0;
    public static final double SteerFF = 0;
  }
}