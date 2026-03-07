package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CANIdConstants;

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

  private final SparkMaxConfig m_leftMotorConfig;
  private final SparkMaxConfig m_middleMotorConfig;
  private final SparkMaxConfig m_rightMotorConfig;

  private double m_targetVelocity_rpm = 0.0;

  // Shot detection
  private double m_lastAvgRPM_rpm = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  public Shooter() {

    m_rightMotor = new SparkMax(CANIdConstants.flywheelRightMotorID, MotorType.kBrushless);
    m_middleMotor = new SparkMax(CANIdConstants.flywheelMiddleMotorID, MotorType.kBrushless);
    m_leftMotor = new SparkMax(CANIdConstants.flywheelLeftMotorID, MotorType.kBrushless);

    m_rightMotorConfig = new SparkMaxConfig();
    m_middleMotorConfig = new SparkMaxConfig();
    m_leftMotorConfig = new SparkMaxConfig();

    m_rightMotorConfig.closedLoop.pid(ShooterConstants.leftP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF,
        ClosedLoopSlot.kSlot0);
    m_middleMotorConfig.closedLoop.pid(ShooterConstants.middleP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF,
        ClosedLoopSlot.kSlot0);
    m_leftMotorConfig.closedLoop.pid(ShooterConstants.rightP, ShooterConstants.I, ShooterConstants.D).velocityFF(ShooterConstants.kF,
        ClosedLoopSlot.kSlot0);

    m_rightMotorConfig.idleMode(IdleMode.kCoast);
    m_middleMotorConfig.idleMode(IdleMode.kCoast);
    m_leftMotorConfig.idleMode(IdleMode.kCoast);

    m_rightMotorConfig.inverted(true);
    m_middleMotorConfig.inverted(false);
    m_leftMotorConfig.inverted(false);

    configureMotor(m_rightMotor, m_rightMotorConfig);
    configureMotor(m_middleMotor, m_middleMotorConfig);
    configureMotor(m_leftMotor, m_leftMotorConfig);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_middleEncoder = m_middleMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();

    m_rightController = m_rightMotor.getClosedLoopController();
    m_middleController = m_middleMotor.getClosedLoopController();
    m_leftController = m_leftMotor.getClosedLoopController();
  }

  public void setPIDValues(double flywheelP, double flywheelI, double flywheelD) {

    // m_forwardMotorConfig.closedLoop
    //     .pid(flywheelP, flywheelI, flywheelD);
    // m_backwardMotorConfig.closedLoop
    //     .pid(flywheelP, flywheelI, flywheelD);
    //Probably not needed.
  }

  private void configureMotor(SparkMax motor, SparkMaxConfig config) {

    motor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setShooterRPM(double targetRPM_rpm) {
    m_targetVelocity_rpm = targetRPM_rpm;

    m_rightController.setSetpoint(m_targetVelocity_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_middleController.setSetpoint(m_targetVelocity_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_leftController.setSetpoint(m_targetVelocity_rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // Speed checks
  public boolean rightAtSpeed() {
    return Math.abs(m_rightEncoder.getVelocity() - m_targetVelocity_rpm) < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean middleAtSpeed() {
    return Math.abs(m_middleEncoder.getVelocity() - m_targetVelocity_rpm) < ShooterConstants.FlywheelRPMTolerance_rpm;
  }

  public boolean leftAtSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - m_targetVelocity_rpm) < ShooterConstants.FlywheelRPMTolerance_rpm;
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

    if ((m_lastAvgRPM_rpm - currentRPM_rpm) > ShooterConstants.shotDipThreshold_rpm) {

      if (m_lastShotTime_sec < 0 ||
          (now_sec - m_lastShotTime_sec) > ShooterConstants.shotMinSeparation_sec) {

        m_shotDetected = true;
        m_lastShotTime_sec = now_sec;
      }
    }

    m_lastAvgRPM_rpm = currentRPM_rpm;
    return m_shotDetected;
  }

  public double getLastShotTime_sec() {
    return m_lastShotTime_sec;
  }

  @Override
  public void periodic() {

    updateShotDetection();

    SmartDashboard.putNumber("Shooter Avg RPM", getAverageRPM());
    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);

    SmartDashboard.putNumber("Right motor RPM", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Middle motor RPM", m_middleEncoder.getVelocity());
    SmartDashboard.putNumber("Left motor RPM", m_leftEncoder.getVelocity());
  }

  public void cancel() {
    m_rightMotor.stopMotor();
    m_middleMotor.stopMotor();
    m_leftMotor.stopMotor();
  }
}
