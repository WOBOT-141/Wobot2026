// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {

  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController steerPID;
  private final CANcoder cancoder;
  private final double encoderOffset;

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
        .smartCurrentLimit(50);

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

  private void syncSteerEncoderToAbsolute() {
    double absoluteRotations = cancoder.getAbsolutePosition().getValueAsDouble();
    double angleDeg = (absoluteRotations - encoderOffset) * 360.0;
    steerMotor.getEncoder().setPosition(angleDeg);
  }

  // ───────────────────────────────────────────────
  // Required methods for Drive subsystem
  // ───────────────────────────────────────────────

  public double get_xPos() {
    return xPosMeters;
  }

  public double get_yPos() {
    return yPosMeters;
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

  public double getAngleDeg() {
    return steerMotor.getEncoder().getPosition();
  }

  // Optional: keep for testing, or remove
  public void setPos(double pos) {
    // If you want to force drive encoder position (usually for testing)
    // driveMotor.getEncoder().setPosition(pos);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Steer Angle " + cancoder.getDeviceID(), getAngleDeg());
  }
}