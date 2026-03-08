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
    public static final int flyWheelRightMotorID = 13;
    public static final int flyWheelMiddlehtMotorID = 14;
    public static final int flyWheelLeftMotorID = 15;
    public static final int feedMotorID = 16;
    public static final int magazineMotorID = 17;
    public static final int hoodMotorID = 20;
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

  public static class DriveConstants {
    public static final double wheelCircumference_in = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;

    public static final double rotationsPerInch_rot = driveGearRatio / wheelCircumference_in;

    public static final double drivekP = 0.0016;
    public static final double drivekI = 0.000068;
    public static final double drivekD = 0;
    public static final double drivekIz = 0;
    public static final double drivekFF = 0.00016500000492669642;

    public static final double topSpeed_fps = 21; // TODO verify this

    public static final double autoRotatekP = 0.4;
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    public static final int driveCurrentLim_amp = 80;

    public static final double driveTargetTolerance = 2.0;
    public static final double visionDrivekP = 0.0032;
    public static final double visionDrivekI = 0.00028;
    public static final double visionDrivekD = 0.00015;
  }

  public static final class FeederConstants {

    // Current limits
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

    // Motor IDs
    public static final int intakeMotorID = 18;
    public static final int intakeRollerMotorID = 19;

    // Current limits
    public static final int freeLimit_amp = 60;

    // Homing (current-based)
    public static final double homingSpeed_perc = -0.20;
    public static final double homingCurrentThreshold_amp = 25.0;
    public static final double homingTimeout_sec = 2.0;

    // Intake positions
    // =========================
    public static final double upAngle_deg = 0.0;
    public static final double downAngle_deg = 95.0;

    /** Allowed error band for position checks */
    public static final double positionTolerance_deg = 3.0;

    // Motion speeds
    public static final double intakeUpSpeed_perc = -1;
    public static final double intakeDownSpeed_perc = 0.65;;

    /** Safety timeout while moving */
    public static final double moveTimeout_sec = 2.0;

    // Brownout protection

    /** Voltage where we start scaling */
    public static final double brownoutStartVoltage_V = 10.8;

    /** Voltage where we heavily limit */
    public static final double brownoutCriticalVoltage_V = 9.8;

    /** Scale factor when in brownout */
    public static final double brownoutScale_perc = 0.8;

    // Encoder conversion

    /**
     * Degrees of intake arm per motor rotation.
     *
     * UPDATE THIS when gear ratio is known.
     * If direct drive: 360
     * If 8:1 gearbox: 360 / 8 = 72
     */
    public static final double degPerMotorRotation_deg = 360.0;
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
    public static final double P = 0.004;
    public static final double I = 0.002;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance_deg = 2;
  }

  public static class ShooterConstants {

    public static final double g_inps2 = Units.metersToInches(9.807);
    public static double drc_in = 23.51; // drc_m = horiz dist rim to target center
    public static double hrm_in = 72; // hrm_m = height of rim
    public static double hm_in = 24.3359; // hm_m = height of muzzle
    // theta = hood angle
    public static double deltaHprm_in = hrm_in - hm_in;

    // PID / Feedforward
    public static final double P = 0.00000;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double kF = 0.00017;

    public static final double hoodP = 0.04;
    public static final double hoodI = 0;
    public static final double hoodD = 0;
    public static final double hoodFF = 0.0;
    
    // Physical properties
    public static final double FlywheelDiameter_in = 4.12;

    // Control tuning
    public static final double FlywheelRPMTolerance_rpm = 100.0;
    public static final double slewRateLimit_rpmps = 4000.0;

    // Current limits
    public static final int stallLimit_amp = 40;
    public static final int freeLimit_amp = 40;

    // Brownout protection
    public static final double BrownoutStartVoltage_V = 10.8;
    public static final double BrownoutCriticalVoltage_V = 9.0;

    // Shot detection via RPM dip
    public static final double shotDipThreshold_rpm = 250.0;
    public static final double shotMinSeparation_sec = 0.08;
  
    public static final double hoodHomeRPM = .1;
    public static final int hoodCurrentLim_amp = 20;
    public static final double hoodRevsPerDegree =  .1099;
    public static final double steerDegPerEncoderTick_deg = hoodRevsPerDegree / 42;

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

  public static class WheelConstants {
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

    public static final double wheelCircumference_in = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;
    public static final double rotationsPerInch = driveGearRatio / wheelCircumference_in;
  }

}