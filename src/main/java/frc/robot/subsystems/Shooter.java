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
import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;

public class Shooter extends SubsystemBase {

  private final SparkMax m_rightMotor;
  private final SparkMax m_middleMotor;
  private final SparkMax m_leftMotor;
  private final SparkMax m_hoodMotor;

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_middleEncoder;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_hoodMotorEncoder;

  private final SparkClosedLoopController m_rightController;
  private final SparkClosedLoopController m_middleController;
  private final SparkClosedLoopController m_leftController;
  private final SparkClosedLoopController m_hoodController;

  private final SparkMaxConfig m_leftMotorConfig;
  private final SparkMaxConfig m_middleMotorConfig;
  private final SparkMaxConfig m_rightMotorConfig;
  private final SparkMaxConfig m_hoodMotorConfig;

  private double m_targetVelocity_ips = 0.0;
  private double m_hoodTargetPosition_deg = ShooterConstants.hoodLowerLimit_deg;
  private boolean m_hoodAtHome = false;

  // Shot detection
  private double m_lastAvgRPM_rpm = 0.0;
  private boolean m_shotDetected = false;
  private double m_lastShotTime_sec = -1.0;

  private String m_hoodPosition_SD = "SETHOODPOS!!"; // SmartDashboard key for hood position

  private double m_startTime_sec = -1;

  public Shooter() {

    SmartDashboard.putNumber("hoodP", ShooterConstants.hoodP);
    SmartDashboard.putNumber("hoodI", ShooterConstants.hoodI);
    SmartDashboard.putNumber("hoodD", ShooterConstants.hoodD);
    SmartDashboard.putNumber(m_hoodPosition_SD, m_hoodTargetPosition_deg);

    m_rightMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless);
    m_middleMotor = new SparkMax(CANIdConstants.flyWheelMiddleMotorID, MotorType.kBrushless);
    m_leftMotor = new SparkMax(CANIdConstants.flyWheelLeftMotorID, MotorType.kBrushless);
    m_hoodMotor = new SparkMax(CANIdConstants.hoodMotorID, MotorType.kBrushless);

    m_rightMotorConfig = new SparkMaxConfig();
    m_middleMotorConfig = new SparkMaxConfig();
    m_leftMotorConfig = new SparkMaxConfig();

    m_rightMotorConfig.inverted(true);
    m_middleMotorConfig.inverted(false);
    m_leftMotorConfig.inverted(false);

    m_rightMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);
    m_middleMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);
    m_leftMotorConfig.encoder.velocityConversionFactor(ShooterConstants.flywheelVelocity_RPMtoIPS);

    m_rightMotorConfig.closedLoop.pid(ShooterConstants.rightP, ShooterConstants.I, ShooterConstants.D).feedForward.kV(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
    m_middleMotorConfig.closedLoop.pid(ShooterConstants.middleP, ShooterConstants.I, ShooterConstants.D).feedForward.kV(ShooterConstants.kF, ClosedLoopSlot.kSlot0);
    m_leftMotorConfig.closedLoop.pid(ShooterConstants.leftP, ShooterConstants.I, ShooterConstants.D).feedForward.kV(ShooterConstants.kF, ClosedLoopSlot.kSlot0);

    m_rightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);
    m_middleMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);
    m_leftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.flywheelCurrentLimit_amp, ShooterConstants.flywheelCurrentLimit_amp);

    m_hoodMotorConfig = new SparkMaxConfig();
    m_hoodMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.hoodCurrentLim_amp)
        .openLoopRampRate(ShooterConstants.hoodSlewRate);
    m_hoodMotorConfig.encoder.positionConversionFactor(ShooterConstants.hoodDegPerMotorRot); 
    m_hoodMotorConfig.closedLoop
        .pid(ShooterConstants.hoodP, ShooterConstants.hoodI, ShooterConstants.hoodD)
        //.velocityFF(ShooterConstants.hood_kF)
        .minOutput(-0.2)
        .maxOutput(0.2);

    m_hoodMotor.configure(m_hoodMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    configureMotor(m_rightMotor, m_rightMotorConfig);
    configureMotor(m_middleMotor, m_middleMotorConfig);
    configureMotor(m_leftMotor, m_leftMotorConfig);
    configureMotor(m_hoodMotor, m_hoodMotorConfig);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_middleEncoder = m_middleMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();
    m_hoodMotorEncoder = m_hoodMotor.getEncoder();

    m_rightController = m_rightMotor.getClosedLoopController();
    m_middleController = m_middleMotor.getClosedLoopController();
    m_leftController = m_leftMotor.getClosedLoopController();
    m_hoodController = m_hoodMotor.getClosedLoopController();
  }

  public void homeHood() {
    double currentTime_sec = Timer.getFPGATimestamp();
    //if ((m_startTime_sec - currentTime_sec) > .05) {
      //m_hoodMotor.set(Constants.ShooterConstants.hoodHomeRPM);
      
      if (m_hoodMotor.getOutputCurrent() > ShooterConstants.hoodCurrentLim_amp) {
        System.out.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
        m_hoodMotor.stopMotor();
        m_hoodMotorEncoder.setPosition(ShooterConstants.hoodUpperLimit_deg);
        m_hoodAtHome = true;
      }
   // }
  }

  public boolean isHoodAtHome() {
    SmartDashboard.putNumber("homePos", m_hoodMotor.getEncoder().getPosition());
    return m_hoodAtHome;
  }

  public void setHoodAtHome(boolean atHome) {
    m_startTime_sec = Timer.getFPGATimestamp();
    m_hoodMotor.set(Constants.ShooterConstants.hoodHomeRPM);
    m_hoodAtHome = atHome;
  }

  public void setHoodAngle(double targetPosition_deg) {
    // System.out.println("---------- setHoodAngle ----------");
    // System.out.println("max "+ShooterConstants.hoodLowerLimit_deg);
    targetPosition_deg = Math.max(targetPosition_deg, ShooterConstants.hoodLowerLimit_deg);
    // System.out.println(targetPosition_deg);
    targetPosition_deg = Math.min(targetPosition_deg, ShooterConstants.hoodUpperLimit_deg);
    // System.out.println(targetPosition_deg);
    m_hoodTargetPosition_deg = targetPosition_deg;
    
    m_hoodController.setSetpoint(m_hoodTargetPosition_deg, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setHoodPositionFromDashboard() {
    double targetPosition_deg = SmartDashboard.getNumber(m_hoodPosition_SD, m_hoodTargetPosition_deg);
    System.out.println("setHoodPositionFromDashboard "+ m_hoodPosition_SD +" "+m_hoodTargetPosition_deg+ " "+targetPosition_deg);
    m_hoodMotor.set(-Constants.ShooterConstants.hoodHomeRPM);
    setHoodAngle(targetPosition_deg);
  }
  
  public void setPIDValues(double hoodP, double hoodI, double hoodD) {
    m_hoodMotorConfig.closedLoop
        .pid(hoodP, hoodI, hoodD);
  }

  private void configureMotor(SparkMax motor, SparkMaxConfig config) {
    motor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setShooterVelocity(double targetVelocity_ips) {
    if (targetVelocity_ips == m_targetVelocity_ips)
      return;

    m_targetVelocity_ips =  targetVelocity_ips * ShooterConstants.flywheelSlipFactor;

    m_rightController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_middleController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_leftController.setSetpoint(m_targetVelocity_ips, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  // Speed checks
  public boolean rightAtSpeed() {
    return Math.abs(m_rightEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean middleAtSpeed() {
    return Math.abs(m_middleEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean leftAtSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - m_targetVelocity_ips) < ShooterConstants.flywheelVelocityTolerance_ips;
  }

  public boolean allAtSpeed() {
    return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
  }

  public boolean hoodAtPosition(){
    return Math.abs(m_hoodMotorEncoder.getPosition() - m_hoodTargetPosition_deg) < ShooterConstants.hoodPositionTolerance_deg;
  }

  public boolean isReadyToShoot() {
    return allAtSpeed() && hoodAtPosition();
  }
  
  public double getAverageVelocity_ips() {
    return (m_rightEncoder.getVelocity()
        + m_middleEncoder.getVelocity()
        + m_leftEncoder.getVelocity()) / 3.0 ;
  }

  // Shot detection via RPM dip
  public boolean updateShotDetection() {

  // TODO convert to ips
    double currentRPM_rpm = getAverageVelocity_ips();
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

  public void cancel() {
    m_rightMotor.stopMotor();
    m_middleMotor.stopMotor();
    m_leftMotor.stopMotor();
    m_hoodMotor.stopMotor();
    m_targetVelocity_ips = 0;
    // m_hoodTargetPosition_deg = ShooterConstants.hoodUpperLimit_deg;
  }

  public void stopHood() {
    m_hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {

    updateShotDetection();

    
    SmartDashboard.putNumber("Shooter Commanded IPS", m_targetVelocity_ips);
    SmartDashboard.putNumber("Shooter Actual IPS", getAverageVelocity_ips());
    SmartDashboard.putNumber("Right Motor SetPoint", m_rightController.getSetpoint());
    SmartDashboard.putBoolean("ShooterHomed", m_hoodAtHome);

    SmartDashboard.putNumber("Hood Commanded Position", m_hoodTargetPosition_deg);
    SmartDashboard.putNumber("Hood Actual Position", m_hoodMotorEncoder.getPosition());
    SmartDashboard.putNumber("Hood Current", m_hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Avg RPM", getAverageVelocity_ips());
    SmartDashboard.putBoolean("Shot Detected", m_shotDetected);

    SmartDashboard.putNumber("Right motor RPM", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Middle motor RPM", m_middleEncoder.getVelocity());
    SmartDashboard.putNumber("Left motor RPM", m_leftEncoder.getVelocity());
  }
}
