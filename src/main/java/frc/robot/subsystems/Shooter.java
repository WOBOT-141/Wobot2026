package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.InchesPerSecond;

import javax.security.auth.login.FailedLoginException;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final SparkMax m_rightMotor;
  private final SparkMax m_middleMotor;
  private final SparkMax m_leftMotor;

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_middleEncoder;
  private final RelativeEncoder m_leftEncoder;

  private final SparkClosedLoopController m_rightController;
  private final SparkClosedLoopController m_middleController;
  private final SparkClosedLoopController m_leftController;

  private double m_targetVelocity_rpm = 0.0;

  // Shot detection
  private double m_lastAvgRPM_rpm = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  public Shooter() {
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkMax m_rightFlywheelMotor;
  private final SparkMax m_middleFlywheelMotor;
  private final SparkMax m_leftFlywheelMotor;
  private final SparkMaxConfig m_rightFlywheelMotorConfig;
  private final SparkMaxConfig m_middleFlywheelMotorConfig;
  private final SparkMaxConfig m_leftFlywheelMotorConfig;

  private final RelativeEncoder m_rightFlywheelEncoder;
  private final RelativeEncoder m_middleFlywheelEncoder;
  private final RelativeEncoder m_leftFlywheelEncoder;
  private final SparkClosedLoopController m_rightFlywheelController;
  private final SparkClosedLoopController m_middleFlywheelController;
  private final SparkClosedLoopController m_leftFlywheelController;

  private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);
  private final SlewRateLimiter m_middleLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);
  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);

  private double m_targetVelocityRPM = 0;

  private double m_shooterStartTime = 0;
  public boolean m_softStartActive = false;

  public boolean m_lastRightAtSpeed = false;
  public boolean m_lastMiddleAtSpeed = false;
  public boolean m_lastLeftAtSpeed = false;

  public Shooter() {
    m_rightFlywheelMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless); // TODO add ids to sparkMaxs
    m_middleFlywheelMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(CANIdConstants.flyWheelLeftMotorID, MotorType.kBrushless);

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_middleFlywheelMotorConfig = new SparkMaxConfig();
    m_leftFlywheelMotorConfig = new SparkMaxConfig();

    m_rightFlywheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.ShooterConstants.stallLimt_amp, Constants.ShooterConstants.stallLimt_amp);
    m_rightFlywheelMotorConfig.closedLoop
        .pid(Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D)
        .velocityFF(Constants.ShooterConstants.FF, ClosedLoopSlot.kSlot0);

    m_middleFlywheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.ShooterConstants.stallLimt_amp, Constants.ShooterConstants.stallLimt_amp);
    m_middleFlywheelMotorConfig.closedLoop
        .pid(Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D)
        .velocityFF(Constants.ShooterConstants.FF, ClosedLoopSlot.kSlot0);

    m_leftFlywheelMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.ShooterConstants.stallLimt_amp, Constants.ShooterConstants.stallLimt_amp);
    m_leftFlywheelMotorConfig.closedLoop
        .pid(Constants.ShooterConstants.P, Constants.ShooterConstants.I, Constants.ShooterConstants.D)
        .velocityFF(Constants.ShooterConstants.FF, ClosedLoopSlot.kSlot0);

    m_rightFlywheelMotor.configure(m_rightFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_middleFlywheelMotor.configure(m_middleFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_leftFlywheelMotor.configure(m_leftFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_rightFlywheelEncoder = m_rightFlywheelMotor.getEncoder();
    m_middleFlywheelEncoder = m_middleFlywheelMotor.getEncoder();
    m_leftFlywheelEncoder = m_middleFlywheelMotor.getEncoder();

    m_rightFlywheelController = m_rightFlywheelMotor.getClosedLoopController();
    m_middleFlywheelController = m_middleFlywheelMotor.getClosedLoopController();
    m_leftFlywheelController = m_leftFlywheelMotor.getClosedLoopController();

    m_rightMotor = new SparkMax(ShooterConstants.flywheelRightMotorID, MotorType.kBrushless);
    m_middleMotor = new SparkMax(ShooterConstants.flywheelMiddleMotorID, MotorType.kBrushless);
    m_leftMotor = new SparkMax(ShooterConstants.flywheelLeftMotorID, MotorType.kBrushless);

    configureMotor(m_rightMotor);
    configureMotor(m_middleMotor);
    configureMotor(m_leftMotor);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_middleEncoder = m_middleMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();

    m_rightController = m_rightMotor.getClosedLoopController();
    m_middleController = m_middleMotor.getClosedLoopController();
    m_leftController = m_leftMotor.getClosedLoopController();
  }

  public void setFlywheelVelocity(double targetVelocity_ips) {
    m_rightFlywheelMotor.set(targetVelocity_ips);
    m_middleFlywheelMotor.set(targetVelocity_ips);
    m_leftFlywheelMotor.set(targetVelocity_ips);

  private void configureMotor(SparkMax motor) {

    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    config.closedLoop
        .pid(ShooterConstants.P,
             ShooterConstants.I,
             ShooterConstants.D)
        .velocityFF(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    motor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setShooterRPM(double targetRPM_rpm) {

    m_targetVelocity_rpm = targetRPM_rpm;

    m_rightController.setSetpoint(targetRPM_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_middleController.setSetpoint(targetRPM_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_leftController.setSetpoint(targetRPM_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // =========================
  // Speed checks
  // =========================

  public boolean rightAtSpeed() {
    return Math.abs(m_rightEncoder.getVelocity() - m_targetVelocity_rpm)
        < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public void advanceFuel() {

  public boolean middleAtSpeed() {
    return Math.abs(m_middleEncoder.getVelocity() - m_targetVelocity_rpm)
        < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean leftAtSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - m_targetVelocity_rpm)
        < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  public double getAverageRPM() {
    return (m_rightEncoder.getVelocity()
          + m_middleEncoder.getVelocity()
          + m_leftEncoder.getVelocity()) / 3.0;
  }
  
  // Shot detection via RPM dip
  public boolean updateShotDetection() {

    double currentRPM_rpm = getAverageRPM();
    double now_sec = Timer.getFPGATimestamp();

    m_shotDetected = false;

    if ((m_lastAvgRPM_rpm - currentRPM_rpm)
        > ShooterConstants.shotDipThreshold_rpm) {

      if (m_lastShotTime_sec < 0 ||
          (now_sec - m_lastShotTime_sec)
              > ShooterConstants.shotMinSeparation_sec) {

        m_shotDetected = true;
        m_lastShotTime_sec = now_sec;
      }
    }

    m_lastAvgRPM_rpm = currentRPM_rpm;
    return m_shotDetected;
  }

  public double getLastShotTime_sec() {
    return m_lastShotTime_sec;
  public void cancel() {
    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
  }

  public void setShooter_ips(double targetVelocity_ips) {
    m_targetVelocityRPM = (targetVelocity_ips * 60) /
        (Math.PI * Constants.ShooterConstants.wheelCircumference_in);
  }

    // spin fly wheel motor one at a time to avoid curent spikes
  public void softStartFlywheel() {
    m_shooterStartTime = Timer.getFPGATimestamp();
    m_softStartActive = true;
  }

  public void unjamFlywheels() {
    m_rightFlywheelMotor.set(-0.75);
    m_middleFlywheelMotor.set(-0.75);
    m_leftFlywheelMotor.set(-0.75);

  }

  public boolean rightAtSpeed() {
    double toleranceRPM = Constants.ShooterConstants.flywheelTolernce_ips;
    return Math.abs(m_rightFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM;
  }

  public boolean middleAtSpeed() {
    double toleranceRPM = Constants.ShooterConstants.flywheelTolernce_ips;
    return Math.abs(m_middleFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM;
  }

  public boolean leftAtSpeed() {
    double toleranceRPM = Constants.ShooterConstants.flywheelTolernce_ips;
    return Math.abs(m_middleFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM;
  }

  public boolean flywheelsAtSpeed() {

    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  @Override
  public void periodic() {

    updateShotDetection();

    SmartDashboard.putNumber("Shooter Avg RPM", getAverageRPM());
    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);
    // This method will be called once per scheduler run
    boolean rightAtSpeed = SmartDashboard.putBoolean("rightAtSpeed", false);
    boolean middleAtspeed = SmartDashboard.putBoolean("middleAtSpeed", false);
    boolean leftAtSpeed = SmartDashboard.putBoolean("leftAtSpeed", false);
    
    double elapsedTime = Timer.getFPGATimestamp() - m_shooterStartTime;

    double rightFlywheelRPM = m_rightLimiter.calculate(m_targetVelocityRPM);
    double middleFlywheelRP = m_middleLimiter.calculate(m_targetVelocityRPM);
    double leftFlywheelRPM = m_middleLimiter.calculate(m_targetVelocityRPM);

    if (elapsedTime > 0.0) {
      m_rightFlywheelController.setSetpoint(rightFlywheelRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    if (elapsedTime > 0.0) {
      m_middleFlywheelController.setSetpoint(middleFlywheelRP, ControlType.kVelocity, ClosedLoopSlot.kSlot0); 
    }
    if (elapsedTime > 0.0) {
      m_leftFlywheelController.setSetpoint(leftFlywheelRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
  }
}
