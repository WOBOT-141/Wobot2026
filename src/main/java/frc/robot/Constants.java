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
    public static final int intakeRollerMotorID = 19;
    public static final int hoodMotorID = 20;
    public static final int intakeElevationLeftMotorID = 21;

    public static final int climberMotorID = 0; //TODO add can ids
    public static final int climbFollowerMotorID = 0;
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


  public static class ControllerOneConstants {
    public static final double RotateFactor = 1.35;
    public static final double DriveXFactor = 1.26;
    public static final double DriveYFactor = 1.23;
    public static final double ThrottleFactor = 1.55;

    public static final double RotateOffset = -0.056375;
    public static final double DriveXOffset = 0.039816 + 0.029202;
    public static final double DriveYOffset = -0.024228;
    public static final double ThrottleOffset = 0;

    public static final double deadbandRot = 0.025;
    public static final double deadbandX = 0.025;
    public static final double deadbandY = 0.025;
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
    public static final int elevationCurrentLimit_amp = 40;
    public static final int rollerCurrentLimit_amp = 60;

    // Homing (current-based)
    public static final double homingSpeed_perc = -0.20;
    public static final double homingCurrentThreshold_amp = 20.0;
    public static final double homingTimeout_sec = 2.0;

    // Intake positions
    public static final double upAngle_deg = 0.0;
    public static final double downAngle_deg = 95.0;

    // Motion speeds
    public static final double intakeRetractSpeed_perc = -0.4;
    public static final double intakeExtendSpeed_perc = 0.4;

    public static final double positionTolerance_deg = 3.0; // Allowed error band for position checks
    public static final double moveTimeout_sec = .20; // Safety timeout while moving

    public static final double twoTurnDistance =1; //Math.abs(-85408.851563-(-84688.945313));

    // Brownout protection
    public static final double brownoutStartVoltage_V = 10.8; // Voltage where we start scaling
    public static final double brownoutCriticalVoltage_V = 9.8; // Voltage where we heavily limit
    public static final double brownoutScale_perc = 0.8; // Scale factor when in brownout
    
    public static final double degPerMotorRotation_deg = 360.0; //TODO change to reflect elevation motor gear ratio
    public static final double intakeRawExtension_rot = 63.4;//63.4

    public static final double SlowSpot = 5;
  }

  public static class OperatorConstants {
    public static final double throttleFloor_perc = 0.2;
  }

  public static class RotateCommandConstants {
    public static final double P = 0.0035;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 3.0;
  }

  public static class RotateVisionConstants {
    public static final double P = 0.004;
    public static final double I = 0.002;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 2;
  }

  public static class ShooterConstants {

    public static final double g_inps2 = Units.metersToInches(9.807);

    public static final double flywheelDiameter_in = 4;
    public static final double flywheelCircum_in = Math.PI * flywheelDiameter_in;
    public static final double flywheelGearRatio = 29 / 24;
    public static final double flywheelVelocity_RPMtoIPS = (flywheelCircum_in * flywheelGearRatio / 60.0) / 2.0; // /2.0 accounts for balls spin
    public static final double flywheelSlipFactor = 1.1; // May need to be scaled with commanded speed
    
    public static final double muzzleHeight_in = 27;
    public static final double hubRimToCenter_in = 23.5;
    public static final double hubRimHeight_in = 72;
    public static final double muzzleToRimHeight_in = hubRimHeight_in - muzzleHeight_in;

    public static final double leftP = 0.0012;
    public static final double middleP = 0.0025;
    public static final double rightP = 0.0015;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double kF = 0.0018;

    public static final double hoodP = 0.05;
    public static final double hoodI = 0;
    public static final double hoodD = 0; //.001
    
    public static final double hoodGearRatio = (10.0 / 62.0) * (50.0 / 319.0);
    public static final double hoodDegPerMotorRot = 360 * hoodGearRatio;
    
    // Control tuning
    public static final double flywheelVelocityTolerance_ips = 25; //15
    public static final double hoodPositionTolerance_deg = 5;  //2
    public static final double hoodSlewRate = 0.5;

    // Current limits
    public static final int flywheelCurrentLimit_amp = 40;
    public static final int hoodCurrentLim_amp = 20;

    // Brownout protection
    public static final double BrownoutStartVoltage_V = 10.8;
    public static final double BrownoutCriticalVoltage_V = 9.0;

    // Shot detection via RPM dip
    public static final double shotDipThreshold_rpm = 250.0;
    public static final double shotMinSeparation_sec = 0.08;
  
    public static final double hoodHomeRPM = 0.1;
    
    public static final double hoodUpperLimit_deg = 89;
    public static final double hoodLowerLimit_deg = 46;

    // Fire command constants
    public static final double firingWaitTime_sec = 0.5;
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