// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

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
  /** Creates a new Shooter.*/
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

 private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(Constants.ShooterConstance.slewRateLimt);
 private final SlewRateLimiter m_middleLimiter = new SlewRateLimiter(Constants.ShooterConstance.slewRateLimt);
 private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(Constants.ShooterConstance.slewRateLimt);

 private double m_targetVelocityRPM = 0;

 private double m_shooterStartTime = 0;
 public boolean m_softStartActive = false;

 public boolean m_lastRightAtSpeed = false;
 public boolean m_lastMiddleAtSpeed = false;
public boolean m_lastLeftAtSpeed = false;


  


 
  public Shooter() {
    m_rightFlywheelMotor = new SparkMax(0, MotorType.kBrushless); //TODO add ids to sparkMaxs
    m_middleFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_middleFlywheelMotorConfig = new SparkMaxConfig();
    m_leftFlywheelMotorConfig = new SparkMaxConfig();

    m_rightFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(Constants.ShooterConstance.stallLimt_amp, Constants.ShooterConstance.stallLimt_amp);
    m_rightFlywheelMotorConfig.closedLoop
    .pid(Constants.ShooterConstance.P, Constants.ShooterConstance.I, Constants.ShooterConstance.D)
    .velocityFF(Constants.ShooterConstance.FF, ClosedLoopSlot.kSlot0);
    
    m_middleFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(Constants.ShooterConstance.stallLimt_amp, Constants.ShooterConstance.stallLimt_amp);
    m_middleFlywheelMotorConfig.closedLoop
    .pid(Constants.ShooterConstance.P, Constants.ShooterConstance.I, Constants.ShooterConstance.D)
    .velocityFF(Constants.ShooterConstance.FF, ClosedLoopSlot.kSlot0);

    m_leftFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(Constants.ShooterConstance.stallLimt_amp, Constants.ShooterConstance.stallLimt_amp);
    m_leftFlywheelMotorConfig.closedLoop
    .pid(Constants.ShooterConstance.P, Constants.ShooterConstance.I, Constants.ShooterConstance.D)
    .velocityFF(Constants.ShooterConstance.FF, ClosedLoopSlot.kSlot0);

    m_rightFlywheelMotor.configure(m_rightFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_middleFlywheelMotor.configure(m_middleFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFlywheelMotor.configure(m_leftFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    m_rightFlywheelEncoder = m_rightFlywheelMotor.getEncoder();
    m_middleFlywheelEncoder = m_middleFlywheelMotor.getEncoder();
    m_leftFlywheelEncoder = m_middleFlywheelMotor.getEncoder();

    m_rightFlywheelController = m_rightFlywheelMotor.getClosedLoopController();
    m_middleFlywheelController = m_middleFlywheelMotor.getClosedLoopController();
    m_leftFlywheelController = m_leftFlywheelMotor.getClosedLoopController();

  }
  
  public void setFlywheelVelocity(double targetVelocity_ips){
    m_rightFlywheelMotor.set(targetVelocity_ips);
    m_middleFlywheelMotor.set(targetVelocity_ips);
    m_leftFlywheelMotor.set(targetVelocity_ips);


    
  }
  
  public void advanceFuel(){

  }

  public void cancel(){
    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
  }

  public void setShooter_ips(double targetVelocity_ips){
    m_targetVelocityRPM =(targetVelocity_ips * 60)/
    (Math.PI * Constants.ShooterConstance.wheelCircumference_in);

    m_shooterStartTime = Timer.getFPGATimestamp();
    m_softStartActive = true;
  }
  public void softStartFlywheel(){

double elapsedTime = Timer.getFPGATimestamp() - m_shooterStartTime;

    double rightFlywheelRPM = m_rightLimiter.calculate(m_targetVelocityRPM);
    double middleFlywheelRP = m_middleLimiter.calculate(m_targetVelocityRPM);
    double leftFlywheelRPM = m_middleLimiter.calculate(m_targetVelocityRPM);

    if(elapsedTime > 0.0){
      m_rightFlywheelController.setSetpoint(m_targetVelocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    }
  
    //spin fly wheel motor one at a time to avoid curent spikes
  }
public boolean rightAtSpeed(){
  double toleranceRPM = Constants.ShooterConstance.flywheelTolernce_ips;
  return Math.abs(m_rightFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM; 
}  
public boolean middleAtSpeed(){
  double toleranceRPM = Constants.ShooterConstance.flywheelTolernce_ips;
  return Math.abs(m_middleFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM; 
}    
public boolean leftAtSpeed(){
  double toleranceRPM = Constants.ShooterConstance.flywheelTolernce_ips;
  return Math.abs(m_middleFlywheelEncoder.getVelocity() - m_targetVelocityRPM) < toleranceRPM; 
} 
public boolean flywheelsAtSpeed(){
  
  return rightAtSpeed() && middleAtSpeed() && leftAtSpeed();
} 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean rightAtSpeed = SmartDashboard.putBoolean("rightAtSpeed", false);
    boolean middleAtspeed = SmartDashboard.putBoolean("middleAtSpeed",false);
    boolean leftAtSpeed = SmartDashboard.putBoolean("leftAtSpeed", false);
  }
}
