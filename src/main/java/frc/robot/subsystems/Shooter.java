// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ShooterConstants;
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
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkMax m_rightFlywheelMotor;
  private final SparkMax m_middleFlywheelMotor;
  private final SparkMax m_leftFlywheelMotor;
  private final SparkMax m_hoodMotor;

  private final SparkMaxConfig m_rightFlywheelMotorConfig;
  private final SparkMaxConfig m_middleFlywheelMotorConfig;
  private final SparkMaxConfig m_leftFlywheelMotorConfig;
  private final SparkMaxConfig m_hoodMotorConfig;

  private final RelativeEncoder m_rightFlywheelEncoder;
  private final RelativeEncoder m_middleFlywheelEncoder;
  private final RelativeEncoder m_leftFlywheelEncoder;
  private final RelativeEncoder m_hoodEncoder;
  private final SparkClosedLoopController m_rightFlywheelController;
  private final SparkClosedLoopController m_middleFlywheelController;
  private final SparkClosedLoopController m_leftFlywheelController;
  private final SparkClosedLoopController m_hoodController;

  private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);
  private final SlewRateLimiter m_middleLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);
  private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(Constants.ShooterConstants.slewRateLimt);

  private double m_targetVelocityRPM = 0;

  private double m_shooterStartTime = 0;
  public boolean m_softStartActive = false;

  public boolean m_lastRightAtSpeed = false;
  public boolean m_lastMiddleAtSpeed = false;
  public boolean m_lastLeftAtSpeed = false;

  public double m_hoodTargetPosition_deg = 0;

  public Shooter() {
 SmartDashboard.putNumber("HoodPosition", m_hoodTargetPosition_deg);

    m_rightFlywheelMotor = new SparkMax(CANIdConstants.flyWheelRightMotorID, MotorType.kBrushless); // TODO add ids to sparkMaxs
    m_middleFlywheelMotor = new SparkMax(CANIdConstants.flyWheelMiddlehtMotorID, MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(CANIdConstants.flyWheelLeftMotorID, MotorType.kBrushless);
    m_hoodMotor = new SparkMax(CANIdConstants.hoodMotorID, MotorType.kBrushless);

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_middleFlywheelMotorConfig = new SparkMaxConfig();
    m_leftFlywheelMotorConfig = new SparkMaxConfig();
    m_hoodMotorConfig = new SparkMaxConfig();

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

    m_hoodMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80, 80);
    m_hoodMotorConfig.closedLoop
        .pid(Constants.ShooterConstants.hoodP, Constants.ShooterConstants.hoodI, Constants.ShooterConstants.hoodD)
        .velocityFF(Constants.ShooterConstants.hoodFF, ClosedLoopSlot.kSlot0);// TODO find out if this would be velocity or position control and then change config and method of setting setpoint accordingly

    m_rightFlywheelMotor.configure(m_rightFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_middleFlywheelMotor.configure(m_middleFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_leftFlywheelMotor.configure(m_leftFlywheelMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_hoodMotor.configure(m_hoodMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_rightFlywheelEncoder = m_rightFlywheelMotor.getEncoder();
    m_middleFlywheelEncoder = m_middleFlywheelMotor.getEncoder();
    m_leftFlywheelEncoder = m_leftFlywheelMotor.getEncoder();
    m_hoodEncoder = m_hoodMotor.getEncoder();

    m_rightFlywheelController = m_rightFlywheelMotor.getClosedLoopController();
    m_middleFlywheelController = m_middleFlywheelMotor.getClosedLoopController();
    m_leftFlywheelController = m_leftFlywheelMotor.getClosedLoopController();
    m_hoodController = m_hoodMotor.getClosedLoopController();

  }

  public void homeHood() {
    m_hoodMotor.set(Constants.ShooterConstants.hoodHomeRPM);
    if(m_hoodMotor.getOutputCurrent() > Constants.ShooterConstants.hoodHomeCurrentLim_amp) {
      m_hoodMotor.stopMotor();
      m_hoodEncoder.setPosition(0);
    }
  }

public void setHoodPosition(double targetPosition_deg) {
    targetPosition_deg = SmartDashboard.getNumber("hoodPosition", targetPosition_deg);
    double targetPositionTicks = targetPosition_deg / ShooterConstants.steerDegPerEncoderTick_deg;
    m_hoodController.setSetpoint(targetPositionTicks, ControlType.kPosition, ClosedLoopSlot.kSlot0);

  }
  public void setFlywheelVelocity(double targetVelocity_ips) {
    m_rightFlywheelMotor.set(targetVelocity_ips);
    m_middleFlywheelMotor.set(targetVelocity_ips);
    m_leftFlywheelMotor.set(targetVelocity_ips);

  }

  public void cancel() {
    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
    m_hoodMotor.stopMotor();
  }

  public void setShooter_ips(double targetVelocity_ips) {
   /*  m_targetVelocityRPM = (targetVelocity_ips * 60) /
        (Math.PI * Constants.ShooterConstants.wheelCircumference_in); */

    m_targetVelocityRPM = targetVelocity_ips * 7.89; //Math was on board in offce told to be flywheel rpm from ips
       
   // m_targetVelocityRPM =  targetVelocity_ips * (60 / ShooterConstants.wheelCircumference_in) * ShooterConstants.flywheelGearRati

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
    // This method will be called once per scheduler run
    boolean rightAtSpeed = SmartDashboard.putBoolean("rightAtSpeed", false);
    boolean middleAtspeed = SmartDashboard.putBoolean("middleAtSpeed", false);
    boolean leftAtSpeed = SmartDashboard.putBoolean("leftAtSpeed", false);

    SmartDashboard.putNumber("HoodEncoderPositon", m_hoodEncoder.getPosition());
    SmartDashboard.putNumber("hoodPosition", m_hoodTargetPosition_deg);
    SmartDashboard.getNumber("hoodPosition", m_hoodTargetPosition_deg);
    
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
}//TODO add hood motor 5 ticks 1 degree max of 49 and min of 89 degreseor or visas versa 
