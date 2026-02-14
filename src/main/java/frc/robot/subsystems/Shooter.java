// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter.*/
 private final SparkMax m_rightFlywheelMotor;
 private final SparkMax m_middleFlywheelMotor;
 private final SparkMax m_leftFlywheelMotor; 
 private final SparkMax m_feedMotor;
 private final SparkMax m_magazineMotor;
 private final SparkMaxConfig m_rightFlywheelMotorConfig;
 private final SparkMaxConfig m_middleFlywheelMotorConfig;
 private final SparkMaxConfig m_leftFlywheelMotorConfig;
 private final SparkMaxConfig m_feedMotorConfig;
 private final SparkMaxConfig m_magazineMotorConfig;

 private final RelativeEncoder m_rightFlywheelEncoder;
 private final RelativeEncoder m_middleFlywheelEncoder;
 private final RelativeEncoder m_leftFlywheelEncoder;
 private final RelativeEncoder m_feedMotorEncoder;
 private final RelativeEncoder m_magazineMotorEncoder;
 private final SparkClosedLoopController m_rightFlywheelLoopController;
 private final SparkClosedLoopController m_middleFlywheelLoopController;
 private final SparkClosedLoopController m_leftFlywheelLoopController;
 private final SparkClosedLoopController m_feedMotorLoopController;
 private final SparkClosedLoopController m_magazineMotorLoopController;


  public Shooter() {
    m_rightFlywheelMotor = new SparkMax(0, MotorType.kBrushless); //TODO add ids to sparkMaxs
    m_middleFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_leftFlywheelMotor = new SparkMax(0,  MotorType.kBrushless);
    m_feedMotor = new SparkMax(0,  MotorType.kBrushless);
    m_magazineMotor = new SparkMax(0, MotorType.kBrushless );

    m_rightFlywheelMotorConfig = new SparkMaxConfig();
    m_middleFlywheelMotorConfig = new SparkMaxConfig();
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

    m_rightFlywheelMotor.configure(m_rightFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_middleFlywheelMotor.configure(m_middleFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFlywheelMotor.configure(m_leftFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_feedMotor.configure(m_feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_magazineMotor.configure(m_magazineMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    m_rightFlywheelEncoder = m_rightFlywheelMotor.getEncoder();
    m_middleFlywheelEncoder = m_middleFlywheelMotor.getEncoder();
    m_leftFlywheelEncoder = m_leftFlywheelMotor.getEncoder();
    m_feedMotorEncoder = m_feedMotor.getEncoder();
    m_magazineMotorEncoder = m_magazineMotor.getEncoder();

    m_rightFlywheelLoopController = m_rightFlywheelMotor.getClosedLoopController();
    m_middleFlywheelLoopController = m_middleFlywheelMotor.getClosedLoopController();
    m_leftFlywheelLoopController = m_leftFlywheelMotor.getClosedLoopController();
    m_feedMotorLoopController = m_feedMotor.getClosedLoopController();
    m_magazineMotorLoopController = m_magazineMotor.getClosedLoopController();


  }
  
  public void setFlywheelVelocity(double targetVelocity_ips){

  }
  
  public void advanceFuel(){

  }

  public void canclel(){
    m_rightFlywheelMotor.stopMotor();
    m_middleFlywheelMotor.stopMotor();
    m_leftFlywheelMotor.stopMotor();
    m_feedMotor.stopMotor();
    m_magazineMotor.stopMotor();
  }

  public void softStartFlywheel(){
    //spin fly wheel motor up one at a time to avoid curent spikes

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
