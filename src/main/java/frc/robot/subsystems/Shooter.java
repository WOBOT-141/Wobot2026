// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.InchesPerSecond;

import javax.security.auth.login.FailedLoginException;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Shooter extends SubsystemBase {
  /** Creates a new Shooter.*/
 private final SparkMax m_rightFlywheelMotor;
 private final SparkMax m_centerFlywheelMotor;
 private final SparkMax m_leftFlywheelMotor; 
 private final SparkMax m_feedMotor;
 private final SparkMax m_magazineMotor;
 private final SparkMaxConfig m_rightFlywheelMotorConfig;
 private final SparkMaxConfig m_centerFlywheelMotorConfig;
 private final SparkMaxConfig m_leftFlywheelMotorConfig;
 private final SparkMaxConfig m_feedMotorConfig;
 private final SparkMaxConfig m_magazineMotorConfig;

 private final RelativeEncoder m_rightFlywheelEncoder;
 private final RelativeEncoder m_centerFlywheelEncoder;
 private final RelativeEncoder m_leftFlywheelEncoder;
 private final RelativeEncoder m_feedMotorEncoder;
 private final RelativeEncoder m_magazineMotorEncoder;
 private final SparkClosedLoopController m_rightFlywheelController;
 private final SparkClosedLoopController m_centerFlywheelController;
 private final SparkClosedLoopController m_leftFlywheelController;
 

  public Shooter() {
    m_rightFlywheelMotor = new SparkMax(0, MotorType.kBrushless); //TODO add ids to sparkMaxs
    m_centerFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_feedMotor = new SparkMax(0,  MotorType.kBrushless);
    m_magazineMotor = new SparkMax(0, MotorType.kBrushless );

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_centerFlywheelMotorConfig = new SparkMaxConfig();
    m_leftFlywheelMotorConfig = new SparkMaxConfig();
    m_feedMotorConfig = new SparkMaxConfig();
    m_magazineMotorConfig = new SparkMaxConfig();
    
    m_rightFlywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(80, 80);

    /** 
    flywheelMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(60, 60);
    flywheelMotorConfig.closedLoop
    .pid(0, 0, 0);
    */

  }
  
  public void setFlywheelVelocity(double targetVelocity_ips){

  }
  
  public void advanceFuel(){

  }

  public void canclel(){
    m_rightFlywheelMotor.stopMotor();
    m_centerFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
    m_feedMotor.stopMotor();
    m_magazineMotor.stopMotor();
  }

  public void softStartFlywheel(){
    //spin fly wheel motor one at a time to avoid curent spikes

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
