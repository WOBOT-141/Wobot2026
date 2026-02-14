// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.InchesPerSecond;

import javax.security.auth.login.FailedLoginException;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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

  public Shooter() {
    m_rightFlywheelMotor = new SparkMax(0, MotorType.kBrushless); //TODO add ids to sparkMaxs
    m_middleFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_middleFlywheelMotorConfig = new SparkMaxConfig();
    m_leftFlywheelMotorConfig = new SparkMaxConfig();

    m_rightFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(80, 80);
    m_rightFlywheelMotorConfig.closedLoop
    .pid(0, 0, 0);
    
    m_middleFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(80, 80);
    m_middleFlywheelMotorConfig.closedLoop
    .pid(0, 0, 0);

    m_leftFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(80, 80);
    m_leftFlywheelMotorConfig.closedLoop
    .pid(0, 0, 0);

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

  public void softStartFlywheel(){
    //spin fly wheel motor one at a time to avoid curent spikes
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
