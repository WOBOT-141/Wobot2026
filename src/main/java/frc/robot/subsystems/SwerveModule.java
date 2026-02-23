// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SteerConstants;

public class SwerveModule extends SubsystemBase {

  private final SparkMax _driveMotor;
  private final SparkMax _steerMotor;
  private SparkClosedLoopController _drivePID;
  private SparkClosedLoopController _steerPID;
  private final CANcoder _cancoder;
  private final RelativeEncoder _wheelDriveEncoder;

  private double _xPos;
  private double _yPos;
  private double m_encoderOffset_rot;

  private int steerMotorID;
  private int driveMotorID;
  private double maxAmpsBL_amp = 0;
  private double maxAmpsBR_amp = 0;
  private double maxAmpsFL_amp = 0;
  private double maxAmpsFR_amp = 0;

  /** Creates a new Wheel. */
  public SwerveModule(int steerId, int driveId, int cancoderId, double absoluteOffset_deg, double xPosition, double yPosition) {
    steerMotorID = steerId;
    driveMotorID = driveId;
    _xPos = xPosition;
    _yPos = yPosition;
    m_encoderOffset_rot = absoluteOffset_deg;

    _cancoder = new CANcoder(cancoderId);
    MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
    _cancoder.getConfigurator().refresh(encoderConfig);
    encoderConfig
        .withAbsoluteSensorDiscontinuityPoint(1);
    _cancoder.getConfigurator().apply(encoderConfig);

    _driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    _steerMotor = new SparkMax(steerId, MotorType.kBrushless);
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50, 50);
    driveConfig.encoder
        .positionConversionFactor(1.0 / WheelConstants.rotationsPerInch)
        .velocityConversionFactor((1.0 / WheelConstants.rotationsPerInch) / 60.0);

    FeedForwardConfig driveFF = new FeedForwardConfig();
    driveFF.kV(WheelConstants.DriveFFKv);

    driveConfig.closedLoop
        .p(WheelConstants.DriveP)
        .i(WheelConstants.DriveI)
        .d(WheelConstants.DriveD)
        .apply(driveFF);

    _drivePID = _driveMotor.getClosedLoopController();

    steerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50, 50);
    steerConfig.encoder
        .positionConversionFactor(SteerConstants.steerDegPerMotorRotation_deg);
    steerConfig.closedLoop
        .pid(WheelConstants.SteerP, WheelConstants.SteerI, WheelConstants.DriveD, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(5, ClosedLoopSlot.kSlot0);

    _steerPID = _steerMotor.getClosedLoopController();

    _driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    _steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    _wheelDriveEncoder = _driveMotor.getEncoder();

    settoZero();
  }

  public double getEncoderOffset_deg() {
    return m_encoderOffset_rot;
  }

  public double get_xPos() {
    return _xPos;
  }

  public double get_yPos() {
    return _yPos;
  }

  public void setEncoderOffset(double m_encoderOffset_rot) {
    this.m_encoderOffset_rot = m_encoderOffset_rot;
  }

  public void set_xPos(double _xPos) {
    this._xPos = _xPos;
  }

  public void set_yPos(double _yPos) {
    this._yPos = _yPos;
  }

  public void setSpeed(double speed) {
    _drivePID.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("The speed of commanding", speed);
  }

  public void setPos(double pos) {
    _drivePID.setSetpoint(pos, ControlType.kPosition);
  }

  public void setAngle(double angle) {
    _steerPID.setSetpoint(angle, ControlType.kPosition);
  }

  public void cancelMovement() {
    _driveMotor.stopMotor();
    _steerMotor.stopMotor();
  }

  public double getSpeed() {
    return _wheelDriveEncoder.getVelocity();
  }

  public double getAbsoluteEncoderAngle_deg() {
    double startAngle_rot = _cancoder.getAbsolutePosition().getValueAsDouble();
    return (startAngle_rot - m_encoderOffset_rot) * 360;
  }

  public double getEncoderDistance() {
    return _wheelDriveEncoder.getPosition();
  }

  public void resetEncoder() {
    _wheelDriveEncoder.setPosition(0);
  }

  public SwerveModuleState bestAngle(SwerveModuleState wheel) {
    double currentAngle_deg = _steerMotor.getEncoder().getPosition();

    int totalRotations_rot = (int) (currentAngle_deg / 360);
    double wantedAngle_deg = wheel.angle.getDegrees() + 180;
    double complimentaryAngle_deg = (wantedAngle_deg + 180) % 360;

    double finalSmallestAngle_deg = 0;
    int direction = 0;

    double deltaCurrent_deg = (wantedAngle_deg + (360 * totalRotations_rot)) - currentAngle_deg;
    double deltaNext_deg = (wantedAngle_deg + (360 * (totalRotations_rot + 1))) - currentAngle_deg;
    double deltaPrevious_deg = (wantedAngle_deg + (360 * (totalRotations_rot - 1))) - currentAngle_deg;

    double deltaCurrentComp_deg = (complimentaryAngle_deg + (360 * totalRotations_rot)) - currentAngle_deg;
    double deltaNextComp_deg = (complimentaryAngle_deg + (360 * (totalRotations_rot + 1))) - currentAngle_deg;
    double deltaPreviousComp_deg = (complimentaryAngle_deg + (360 * (totalRotations_rot - 1))) - currentAngle_deg;

    double smallestAngle_deg = deltaPrevious_deg;
    if (Math.abs(smallestAngle_deg) > Math.abs(deltaCurrent_deg)) {
      smallestAngle_deg = deltaCurrent_deg;
    }
    if (Math.abs(smallestAngle_deg) > Math.abs(deltaNext_deg)) {
      smallestAngle_deg = deltaNext_deg;
    }

    double smallestAngleComp_deg = deltaPreviousComp_deg;
    if (Math.abs(smallestAngleComp_deg) > Math.abs(deltaCurrentComp_deg)) {
      smallestAngleComp_deg = deltaCurrentComp_deg;
    }
    if (Math.abs(smallestAngleComp_deg) > Math.abs(deltaNextComp_deg)) {
      smallestAngleComp_deg = deltaNextComp_deg;
    }

    if (Math.abs(smallestAngle_deg) <= Math.abs(smallestAngleComp_deg)) {
      finalSmallestAngle_deg = smallestAngle_deg;
      direction = -1;
    } else {
      finalSmallestAngle_deg = smallestAngleComp_deg;
      direction = 1;
    }

    return new SwerveModuleState(
        wheel.speedMetersPerSecond * direction,
        new Rotation2d(Units.degreesToRadians(currentAngle_deg + finalSmallestAngle_deg)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoderDist", getEncoderDistance());
    SmartDashboard.putNumber("speeed", getSpeed());

    steerMotorPosDebug();
    absEncoderDebug();
    tractionMotorAMPDebug();
  }

  public void tractionMotorAMPDebug() {
    double outputCurrent_amp = _driveMotor.getOutputCurrent();

    switch (driveMotorID) {
      case Constants.CANIdConstants.backLeftDriveMotorID:
        SmartDashboard.putNumber("Back Left Amps: ", outputCurrent_amp);
        if (outputCurrent_amp > maxAmpsBL_amp) {
          maxAmpsBL_amp = outputCurrent_amp;
        }
        SmartDashboard.putNumber("Back Left Max Amps: ", maxAmpsBL_amp);
        break;
      case Constants.CANIdConstants.backRightDriveMotorID:
        SmartDashboard.putNumber("Back Right Amps: ", outputCurrent_amp);
        if (outputCurrent_amp > maxAmpsBR_amp) {
          maxAmpsBR_amp = outputCurrent_amp;
        }
        SmartDashboard.putNumber("Back Right Max Amps: ", maxAmpsBR_amp);
        break;
      case Constants.CANIdConstants.frontLeftDriveMotorID:
        SmartDashboard.putNumber("Front Left Amps: ", outputCurrent_amp);
        if (outputCurrent_amp > maxAmpsFL_amp) {
          maxAmpsFL_amp = outputCurrent_amp;
        }
        SmartDashboard.putNumber("Front Left Max Amps: ", maxAmpsFL_amp);
        break;
      case Constants.CANIdConstants.frontRightDriveMotorID:
        SmartDashboard.putNumber("Front Right Amps: ", outputCurrent_amp);
        if (outputCurrent_amp > maxAmpsFR_amp) {
          maxAmpsFR_amp = outputCurrent_amp;
        }
        SmartDashboard.putNumber("Front Right Max Amps: ", maxAmpsFR_amp);
        break;
    }
  }

  public void steerMotorPosDebug() {
    double currentAngle_deg = (_steerMotor.getEncoder().getPosition());
    switch (steerMotorID) {

      case Constants.CANIdConstants.backLeftSteerMotorID:
        SmartDashboard.putNumber("Neo position Back Left", currentAngle_deg);
        // System.out.printf("Neo position Back Left: %.3f\n", currentAngle_deg);
        break;
      case Constants.CANIdConstants.backRightSteerMotorID:
        SmartDashboard.putNumber("Neo position Back Right", currentAngle_deg);
        // System.out.printf("Neo position Back Right: %.3f\n", currentAngle_deg);
        break;
      case Constants.CANIdConstants.frontLeftSteerMotorID:
        SmartDashboard.putNumber("Neo position Front Left", currentAngle_deg);
        // System.out.printf("Neo position Front Left: %.3f\n", currentAngle_deg);
        break;
      case Constants.CANIdConstants.frontRightSteerMotorID:
        SmartDashboard.putNumber("Neo position Front Right", currentAngle_deg);
        // System.out.printf("Neo position Front Right: %.3f\n", currentAngle_deg);
        break;
    }
  }

  public void absEncoderDebug() {
    // double offsetAngle = getEncoderOffset_deg();
    double startAngle_deg = _cancoder.getAbsolutePosition().getValueAsDouble();
    // double startAngle_deg = ggetEncoderOffset_deg();
    switch (steerMotorID) {

      case Constants.CANIdConstants.backLeftSteerMotorID:
        SmartDashboard.putNumber("Back Left Cancoder: ", startAngle_deg);
        break;
      case Constants.CANIdConstants.backRightSteerMotorID:
        SmartDashboard.putNumber("Back Right Cancoder: ", startAngle_deg);
        break;
      case Constants.CANIdConstants.frontLeftSteerMotorID:
        SmartDashboard.putNumber("Front Left Cancoder: ", startAngle_deg);
        break;
      case Constants.CANIdConstants.frontRightSteerMotorID:
        SmartDashboard.putNumber("Front Right Cancoder: ", startAngle_deg);
        break;
      default:
        System.out.println("default: " + steerMotorID);
    }
  }

  public void settoZero() {
    double absoluteAngle = getAbsoluteEncoderAngle_deg();
    _steerMotor.getEncoder().setPosition(absoluteAngle);
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

    // TODO double check all these numbers
    public static final double wheelCircumference_in = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;
    public static final double rotationsPerInch = driveGearRatio / wheelCircumference_in;
  }
}