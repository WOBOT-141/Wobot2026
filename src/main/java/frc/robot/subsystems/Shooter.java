// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

public class Shooter extends SubsystemBase {
  
  // Hardware
  private final SparkMax m_rightFlywheelMotor;
  private final SparkMax m_middleFlywheelMotor;
  private final SparkMax m_leftFlywheelMotor;

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_middleEncoder;
  private final RelativeEncoder m_leftEncoder;

  private final SparkClosedLoopController m_rightController;
  private final SparkClosedLoopController m_middleController;
  private final SparkClosedLoopController m_leftController;
  
  // Slew limiting (RPM/sec)
  private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(slewRateLimit_rpmps);
  private final SlewRateLimiter m_middleLimiter = new SlewRateLimiter(slewRateLimit_rpmps);
  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(slewRateLimit_rpmps);
 
  // State
  private double m_targetVelocityRPM = 0.0;

  private double m_softStartStartTime_sec = 0.0;
  private boolean m_softStartActive = false;

  // Shot detection
  private double m_lastAverageRPM = 0.0;
  private double m_lastShotTime_sec = -1.0;

  // Dashboard change detection
  private boolean m_lastRightAtSpeed = false;
  private boolean m_lastMiddleAtSpeed = false;
  private boolean m_lastLeftAtSpeed = false;
  private boolean m_lastAllAtSpeed = false;
  
  // Constructor
  public Shooter() {

    m_rightFlywheelMotor = new SparkMax(flywheelRightMotorID, MotorType.kBrushless);
    m_middleFlywheelMotor = new SparkMax(flywheelMiddleMotorID, MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(flywheelLeftMotorID, MotorType.kBrushless);

    configureMotor(m_rightFlywheelMotor);
    configureMotor(m_middleFlywheelMotor);
    configureMotor(m_leftFlywheelMotor);

    m_rightEncoder = m_rightFlywheelMotor.getEncoder();
    m_middleEncoder = m_middleFlywheelMotor.getEncoder();
    m_leftEncoder = m_leftFlywheelMotor.getEncoder();

    m_rightController = m_rightFlywheelMotor.getClosedLoopController();
    m_middleController = m_middleFlywheelMotor.getClosedLoopController();
    m_leftController = m_leftFlywheelMotor.getClosedLoopController();
  }

 
  // Motor configuration
  private void configureMotor(SparkMax motor) {

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(stallLimit_amp, freeLimit_amp);

    config.closedLoop
        .pid(P, I, D)
        .velocityFF(kF, ClosedLoopSlot.kSlot0);

    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
  
  // Public API
  /** Command shooter using IPS */
  public void setShooterVelocity_ips(double targetVelocity_ips) {

    m_targetVelocityRPM =
        (targetVelocity_ips * 60.0) /
        (Math.PI * FlywheelDiameter_in);

    m_softStartStartTime_sec = Timer.getFPGATimestamp();
    m_softStartActive = true;
  }

  public double getAverageRPM() {
    return (m_rightEncoder.getVelocity()
        + m_middleEncoder.getVelocity()
        + m_leftEncoder.getVelocity()) / 3.0;
  }

  public boolean rightAtSpeed() {
    return Math.abs(
        m_rightEncoder.getVelocity() - m_targetVelocityRPM)
        < FlywheelRPMTolerance_rpm;
  }

  public boolean middleAtSpeed() {
    return Math.abs(
        m_middleEncoder.getVelocity() - m_targetVelocityRPM)
        < FlywheelRPMTolerance_rpm;
  }

  public boolean leftAtSpeed() {
    return Math.abs(
        m_leftEncoder.getVelocity() - m_targetVelocityRPM)
        < FlywheelRPMTolerance_rpm;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  /** Shot detection via RPM dip */
  public boolean didJustShoot() {

    double avgRPM = getAverageRPM();
    double now_sec = Timer.getFPGATimestamp();

    boolean dipDetected =
        (m_lastAverageRPM - avgRPM) > shotDipThreshold_rpm;

    boolean separationOK =
        (now_sec - m_lastShotTime_sec) > shotMinSeparation_sec;

    if (dipDetected && separationOK) {
      m_lastShotTime_sec = now_sec;
      m_lastAverageRPM = avgRPM;
      return true;
    }

    m_lastAverageRPM = avgRPM;
    return false;
  }

  public void cancel() {
    m_softStartActive = false;

    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
  }
  
  // Soft start (staggered)
  private void softStartFlywheel() {

    double elapsed_sec = Timer.getFPGATimestamp() - m_softStartStartTime_sec;
    double rightRPM = m_rightLimiter.calculate(m_targetVelocityRPM);
    double middleRPM = m_middleLimiter.calculate(m_targetVelocityRPM);
    double leftRPM = m_leftLimiter.calculate(m_targetVelocityRPM);

    if (elapsed_sec > 0.0) {
      m_rightController.setSetpoint(rightRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    if (elapsed_sec > 0.05) {
      m_middleController.setSetpoint(middleRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    if (elapsed_sec > 0.10) {
      m_leftController.setSetpoint(leftRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
  }
  
  // Brownout scaling helper
  private double getBrownoutScale_perc() {

    double voltage_V = RobotController.getBatteryVoltage();

    if (voltage_V < BrownoutCriticalVoltage_V) {
      return 0.0;
    }

    if (voltage_V < BrownoutStartVoltage_V) {
      return 0.6;
    }

    return 1.0;
  }
  
  @Override
  public void periodic() {

    if (m_softStartActive) {
      softStartFlywheel();
    }

    // Dashboard numbers
    SmartDashboard.putNumber(
        "Shooter Target RPM",
        m_targetVelocityRPM);

    SmartDashboard.putNumber(
        "Shooter Right RPM",
        m_rightEncoder.getVelocity());

    SmartDashboard.putNumber(
        "Shooter Middle RPM",
        m_middleEncoder.getVelocity());

    SmartDashboard.putNumber(
        "Shooter Left RPM",
        m_leftEncoder.getVelocity());

    // Change-only booleans
    boolean rightNow = rightAtSpeed();
    boolean middleNow = middleAtSpeed();
    boolean leftNow = leftAtSpeed();
    boolean allNow = allAtSpeed();

    if (rightNow != m_lastRightAtSpeed) {
      SmartDashboard.putBoolean("Right At Speed", rightNow);
      m_lastRightAtSpeed = rightNow;
    }

    if (middleNow != m_lastMiddleAtSpeed) {
      SmartDashboard.putBoolean("Middle At Speed", middleNow);
      m_lastMiddleAtSpeed = middleNow;
    }

    if (leftNow != m_lastLeftAtSpeed) {
      SmartDashboard.putBoolean("Left At Speed", leftNow);
      m_lastLeftAtSpeed = leftNow;
    }

    if (allNow != m_lastAllAtSpeed) {
      SmartDashboard.putBoolean("Shooter All At Speed", allNow);
      m_lastAllAtSpeed = allNow;
    }
  }
}
