package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;

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

  // ===============================
  // Motors
  // ===============================
  private final SparkMax m_rightFlywheelMotor;
  private final SparkMax m_middleFlywheelMotor;
  private final SparkMax m_leftFlywheelMotor;

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_middleEncoder;
  private final RelativeEncoder m_leftEncoder;

  private final SparkClosedLoopController m_rightController;
  private final SparkClosedLoopController m_middleController;
  private final SparkClosedLoopController m_leftController;

  // ===============================
  // Slew Rate Limiters
  // ===============================
  private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(ShooterConstants.slewRateLimit);

  private final SlewRateLimiter m_middleLimiter = new SlewRateLimiter(ShooterConstants.slewRateLimit);

  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(ShooterConstants.slewRateLimit);

  // ===============================
  // State Variables
  // ===============================
  private double m_targetVelocityRPM = 0;

  private double m_shooterStartTime = 0;
  private boolean m_softStartActive = false;

  // Edge detection for dashboard updates
  private boolean m_lastRightAtSpeed = false;
  private boolean m_lastMiddleAtSpeed = false;
  private boolean m_lastLeftAtSpeed = false;
  private boolean m_lastAllAtSpeed = false;

  // ===============================
  // Constructor
  // ===============================
  public Shooter() {

    m_rightFlywheelMotor = new SparkMax(ShooterConstants.flywheelRightMotorID, MotorType.kBrushless);

    m_middleFlywheelMotor = new SparkMax(ShooterConstants.flywheelRightMotorID, MotorType.kBrushless);

    m_leftFlywheelMotor = new SparkMax(ShooterConstants.flywheelRightMotorID, MotorType.kBrushless);

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

  // ===============================
  // Motor Configuration
  // ===============================
  private void configureMotor(SparkMax motor) {

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.stallLimit_amp,ShooterConstants.freeLimit_amp);

    config.closedLoop
        .pid(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    motor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // ===============================
  // Start Shooter (called once)
  // ===============================
  public void startSoftStart(double targetVelocity_ips) {

    m_targetVelocityRPM = (targetVelocity_ips * 60.0) /
        (Math.PI * ShooterConstants.FlywheelDiameterInches);

    m_shooterStartTime = Timer.getFPGATimestamp();
    m_softStartActive = true;

    // Reset limiters to current velocity
    m_rightLimiter.reset(m_rightEncoder.getVelocity());
    m_middleLimiter.reset(m_middleEncoder.getVelocity());
    m_leftLimiter.reset(m_leftEncoder.getVelocity());
  }

  // ===============================
  // Soft Start + Brownout Protection
  // ===============================
  private void softStartFlywheel() {

    double elapsed = Timer.getFPGATimestamp() - m_shooterStartTime;
    double batteryVoltage = RobotController.getBatteryVoltage();

    SmartDashboard.putNumber("Battery Voltage", batteryVoltage);

    double voltageScale = 1.0;

    if (batteryVoltage < ShooterConstants.BrownoutStartVoltage) {
      voltageScale = batteryVoltage / ShooterConstants.BrownoutStartVoltage;
    }

    if (batteryVoltage < ShooterConstants.BrownoutCriticalVoltage) {
      cancel();
      SmartDashboard.putBoolean("Shooter Brownout Protect", true);
      return;
    } else {
      SmartDashboard.putBoolean("Shooter Brownout Protect", false);
    }

    double rightRPM = m_rightLimiter.calculate(m_targetVelocityRPM) * voltageScale;

    double middleRPM = m_middleLimiter.calculate(m_targetVelocityRPM) * voltageScale;

    double leftRPM = m_leftLimiter.calculate(m_targetVelocityRPM) * voltageScale;

    // Staggered startup
    if (elapsed > 0.0) {
      m_rightController.setSetpoint(rightRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    if (elapsed > 0.05) {
      m_middleController.setSetpoint(middleRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    if (elapsed > 0.10) {
      m_leftController.setSetpoint(leftRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    // Stop ramp once stable
    if (allAtSpeed()) {
      m_softStartActive = false;
    }
  }

  // ===============================
  // Speed Checks
  // ===============================
  public boolean rightAtSpeed() {
    return Math.abs(
        m_rightEncoder.getVelocity() - m_targetVelocityRPM) < ShooterConstants.FlywheelRPMTolerance;
  }

  public boolean middleAtSpeed() {
    return Math.abs(
        m_middleEncoder.getVelocity() - m_targetVelocityRPM) < ShooterConstants.FlywheelRPMTolerance;
  }

  public boolean leftAtSpeed() {
    return Math.abs(
        m_leftEncoder.getVelocity() - m_targetVelocityRPM) < ShooterConstants.FlywheelRPMTolerance;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed()
        && middleAtSpeed()
        && leftAtSpeed();
  }

  // ===============================
  // Stop Shooter
  // ===============================
  public void cancel() {

    m_softStartActive = false;

    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
  }

  // ===============================
  // Periodic
  // ===============================
  @Override
  public void periodic() {

    if (m_softStartActive) {
      softStartFlywheel();
    }

    SmartDashboard.putNumber("Shooter Target RPM", m_targetVelocityRPM);
    SmartDashboard.putNumber("Shooter Right RPM", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Middle RPM", m_middleEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Left RPM", m_leftEncoder.getVelocity());

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
      SmartDashboard.putBoolean("All At Speed", allNow);
      m_lastAllAtSpeed = allNow;
    }
  }
}
