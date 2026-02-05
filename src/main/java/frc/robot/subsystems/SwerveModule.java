// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
=======
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController steerPID;
  private final CANcoder cancoder;
  private final double encoderOffset;

<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
  private  double _xPos;
  private  double _yPos;
  private  double encoderOffset;

  private int steerMotorID;
  private int driveMotorID;
  private double maxAmpsBL = 0;
  private double maxAmpsBR = 0;
  private double maxAmpsFL = 0;
  private double maxAmpsFR = 0;

  private SimpleMotorFeedforward feedForward;

  /** Creates a new Wheel. */
  public SwerveModule(int steerId, int driveId, int cancoderId, double absoluteOffset, boolean motorInvert,
    boolean encoderInvert, double xPosition, double yPosition) {
    steerMotorID = steerId;
    driveMotorID = driveId;
    _xPos = xPosition;
    _yPos = yPosition;
    encoderOffset = absoluteOffset;

    //Ks = min volts to make motor barely move, Kv = got number from neo documentation 
    feedForward = new SimpleMotorFeedforward(0.12, .473);

=======
  // Module position relative to robot center (in meters)
  private final double xPosMeters;  // right = positive
  private final double yPosMeters;  // forward = positive

  @SuppressWarnings("deprecation")

  public Wheel(
      int steerId,
      int driveId,
      int cancoderId,
      double absoluteOffsetRotations,
      boolean driveInvert,
      double xPosMeters,          // ← NEW
      double yPosMeters           // ← NEW
  ) {
    this.encoderOffset = absoluteOffsetRotations;
    this.xPosMeters = xPosMeters;
    this.yPosMeters = yPosMeters;
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java

    // Absolute Encoder
    cancoder = new CANcoder(cancoderId);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Drive motor
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveInvert)
        .idleMode(IdleMode.kBrake)
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
        .smartCurrentLimit(50, 50);
    driveConfig.encoder
        .positionConversionFactor(1.0/WheelConstants.rotationsPerInch)
        .velocityConversionFactor((1.0/WheelConstants.rotationsPerInch)/60.0);
    driveConfig.closedLoop
        .pidf(WheelConstants.DriveP, WheelConstants.DriveI, WheelConstants.DriveD, WheelConstants.DriveFF);
    _drivePID = _driveMotor.getClosedLoopController();
=======
        .smartCurrentLimit(50);
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java

    // Steer motor
    steerMotor = new SparkMax(steerId, MotorType.kBrushless);
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);

    // Apply configs
    // It did not look like there is a new method to do this yet. The depereciated version was left.
    driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    steerMotor.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    drivePID = driveMotor.getClosedLoopController();
    steerPID = steerMotor.getClosedLoopController();

    syncSteerEncoderToAbsolute();
  }

<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
  public double getEncoderOffset() {
    return encoderOffset;
=======
  private void syncSteerEncoderToAbsolute() {
    double absoluteRotations = cancoder.getAbsolutePosition().getValueAsDouble();
    double angleDeg = (absoluteRotations - encoderOffset) * 360.0;
    steerMotor.getEncoder().setPosition(angleDeg);
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java
  }

  // ───────────────────────────────────────────────
  // Required methods for Drive subsystem
  // ───────────────────────────────────────────────

  public double get_xPos() {
    return xPosMeters;
  }

  public double get_yPos() {
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
    return _yPos;
  }

  public void setEncoderOffset(double encoderOffset) {
    this.encoderOffset = encoderOffset;
  }

  public void set_xPos(double _xPos) {
    this._xPos = _xPos;
  }

  public void set_yPos(double _yPos) {
    this._yPos = _yPos;
  }

  public void setSpeed(double speed) {
    _drivePID.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward.calculate(speed));
    SmartDashboard.putNumber("The speed of commanding", speed);
  }

  public void setPos(double pos){
    _drivePID.setSetpoint(pos, ControlType.kPosition);
  }

  public void setAngle(double angle) {
    _steerPID.setSetpoint(angle, ControlType.kPosition);
=======
    return yPosMeters;
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java
  }

  public void cancelMovement() {
    drivePID.setSetpoint(0, SparkBase.ControlType.kVelocity);
    // Optionally keep steer angle where it is, or set to 0 / rest
    // steerPID.setSetpoint(0, SparkBase.ControlType.kPosition);
  }

  /**
   * Returns optimized SwerveModuleState (minimizes rotation < 90°)
   * Compatible with WPILib 2026+ .optimize() behavior
   */
  public SwerveModuleState bestAngle(SwerveModuleState desiredState) {
    Rotation2d currentAngle = Rotation2d.fromDegrees(getAngleDeg());
    desiredState.optimize(currentAngle);   // mutates the state
    return desiredState;
  }

  // ───────────────────────────────────────────────
  // Existing / renamed methods
  // ───────────────────────────────────────────────

  public void setSpeed(double speedMetersPerSecond) {
    drivePID.setSetpoint(speedMetersPerSecond, SparkBase.ControlType.kVelocity);
  }

  public void setAngle(double angleDeg) {
    steerPID.setSetpoint(angleDeg, SparkBase.ControlType.kPosition);
  }
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
  public void resetEncoder(){
    _wheelDriveEncoder.setPosition(0);
=======

  public double getAngleDeg() {
    return steerMotor.getEncoder().getPosition();
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java
  }

  // Optional: keep for testing, or remove
  public void setPos(double pos) {
    // If you want to force drive encoder position (usually for testing)
    // driveMotor.getEncoder().setPosition(pos);
  }

  @Override
  public void periodic() {
<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/SwerveModule.java
    SmartDashboard.putNumber("encoderDist", getEncoderDistance());
    SmartDashboard.putNumber("speeed", getSpeed());

    // steerMotorPosDebug();
    // absEncoderDebug();
    // steerMotorAMPDebug();
  }

  public void steerMotorAMPDebug() {
    double outputAMPs = _driveMotor.getOutputCurrent();

    switch (driveMotorID) {
      case Constants.SteerConstants.backLeftDriveMotorID:
        SmartDashboard.putNumber("Back Left Amps: ", outputAMPs);
        if (outputAMPs > maxAmpsBL) {
          maxAmpsBL = outputAMPs;
        }
        SmartDashboard.putNumber("Back Left Max Amps: ", maxAmpsBL);
        break;
      case Constants.SteerConstants.backRightDriveMotorID:
        SmartDashboard.putNumber("Back Right Amps: ", outputAMPs);
        if (outputAMPs > maxAmpsBR) {
          maxAmpsBR = outputAMPs;
        }
        SmartDashboard.putNumber("Back Right Max Amps: ", maxAmpsBR);
        break;
      case Constants.SteerConstants.frontLeftDriveMotorID:
        SmartDashboard.putNumber("Front Left Amps: ", outputAMPs);
        if (outputAMPs > maxAmpsFL) {
          maxAmpsFL = outputAMPs;
        }
        SmartDashboard.putNumber("Front Left Max Amps: ", maxAmpsFL);
        break;
      case Constants.SteerConstants.frontRightDriveMotorID:
        SmartDashboard.putNumber("Front Right Amps: ", outputAMPs);
        if (outputAMPs > maxAmpsFR) {
          maxAmpsFR = outputAMPs;
        }
        SmartDashboard.putNumber("Front Right Max Amps: ", maxAmpsFR);
        break;
    }
  }

  public void steerMotorPosDebug() {
    double currentAngle = (_steerMotor.getEncoder().getPosition());
    switch (steerMotorID) {
      case Constants.SteerConstants.backLeftSteerMotorID:
        SmartDashboard.putNumber("Neo position Back Left", currentAngle);
        break;
      case Constants.SteerConstants.backRightSteerMotorID:
        SmartDashboard.putNumber("Neo position Back Right", currentAngle);
        break;
      case Constants.SteerConstants.frontLeftSteerMotorID:
        SmartDashboard.putNumber("Neo position Front Left", currentAngle);
        break;
      case Constants.SteerConstants.frontRightSteerMotorID:
        SmartDashboard.putNumber("Neo position Front Right", currentAngle);
        break;
    }
  }

  public void absEncoderDebug() {
    double offsetAngle = 0;
    double startAngle = _cancoder.getAbsolutePosition().getValueAsDouble();
    switch (steerMotorID) {
      case Constants.SteerConstants.backLeftSteerMotorID:
        offsetAngle = getEncoderOffset();
        SmartDashboard.putNumber("Back Left Cancoder: ", (startAngle - offsetAngle) * 360);
        break;
      case Constants.SteerConstants.backRightSteerMotorID:
        offsetAngle = getEncoderOffset();
        SmartDashboard.putNumber("Back Right Cancoder: ", (startAngle - offsetAngle) * 360);
        break;
      case Constants.SteerConstants.frontLeftSteerMotorID:
        offsetAngle = getEncoderOffset();
        SmartDashboard.putNumber("Front Left Cancoder: ", (startAngle - offsetAngle) * 360);
        break;
      case Constants.SteerConstants.frontRightSteerMotorID:
        offsetAngle = getEncoderOffset();
        SmartDashboard.putNumber("Front Right Cancoder: ", (startAngle - offsetAngle) * 360);
        break;
      default:
        System.out.println("default: " + steerMotorID);
    }
  }

  public void settoZero() {
    double startAngle = _cancoder.getAbsolutePosition().getValueAsDouble();
    double offsetAngle = encoderOffset;
    double absoluteAngle = (startAngle - offsetAngle) * 360;
    _steerMotor.getEncoder().setPosition(absoluteAngle);
  }

  public static class WheelConstants {
    public static final double DriveP = 0.0000075; //.0150
    public static final double DriveI = 0;
    public static final double DriveD = 0;
    public static final double DriveIz = 0;
    public static final double DriveFF = 1.0/473 ; //.00060

    public static final double SteerP = 0.025;
    public static final double SteerI = 0;
    public static final double SteerD = 0;
    public static final double SteerIz = 0;
    public static final double SteerFF = 0;

    //TODO double check all these numbers 
    public static final double wheelCircumference = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;
    public static final double rotationsPerInch = driveGearRatio / wheelCircumference;
=======
    SmartDashboard.putNumber("Steer Angle " + cancoder.getDeviceID(), getAngleDeg());
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Wheel.java
  }
}