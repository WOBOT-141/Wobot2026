// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.FeederConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class MagazineAndFeed extends SubsystemBase {
  /** Creates a new Shooter.*/
 private final SparkMax m_feedMotor;
 private final SparkMax m_magazineMotor;
 private final SparkMaxConfig m_feedMotorConfig;
 private final SparkMaxConfig m_magazineMotorConfig;

 private final RelativeEncoder m_feedMotorEncoder;
 private final RelativeEncoder m_magazineMotorEncoder;
 private final SparkClosedLoopController m_feedMotorController;
 private final SparkClosedLoopController m_magazineMotorController;

  public MagazineAndFeed() {
    m_feedMotor = new SparkMax(CANIdConstants.feedMotorID,  MotorType.kBrushless);
    m_magazineMotor = new SparkMax(CANIdConstants.magazineMotorID, MotorType.kBrushless );

    m_feedMotorConfig = new SparkMaxConfig();
    m_magazineMotorConfig = new SparkMaxConfig();
    
    m_feedMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(FeederConstants.feedMotorFreeLimit_amp, FeederConstants.feedMotorFreeLimit_amp);
    // m_feedMotorConfig.closedLoop
    // .pid(0, 0, 0);

    m_magazineMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(FeederConstants.magazineMotorFreeLimit_amp, FeederConstants.magazineMotorFreeLimit_amp);
    // m_magazineMotorConfig.closedLoop
    // .pid(0, 0, 0);
    
    m_feedMotor.configure(m_feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_feedMotor.configure(m_feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_feedMotorEncoder = m_feedMotor.getEncoder();
    m_magazineMotorEncoder = m_magazineMotor.getEncoder();
    
    m_feedMotorController = m_feedMotor.getClosedLoopController();
    m_magazineMotorController = m_magazineMotor.getClosedLoopController();

  }

  public void setFeedVelocity(double targetVelocity_perc){
    m_feedMotor.set(targetVelocity_perc);
    m_magazineMotor.set(targetVelocity_perc);
    
  }
  
  public void advanceFuel(){

  }

  public void cancel(){
    m_feedMotor.stopMotor();
    m_magazineMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
