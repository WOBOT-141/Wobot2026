package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Climber extends SubsystemBase {
    private SparkMax m_leftMotor1;
    private SparkMax m_leftMotor2;
    private SparkMax m_rightMotor1;
    private SparkMax m_rightMotor2;
    private int leftServo1;
    private int leftServo2; // couldnt find much on servos
    private int rightServo1;
    private int rightServo2;
    private SparkMaxConfig m_leftMotor1Config;
    private SparkMaxConfig m_leftMotor2Config; 
    private SparkMaxConfig m_rightMotor1Config;
    private SparkMaxConfig m_rightMotor2Config;

    public Climber() {
        // initialize motors, encoders, configs, etc. here
        m_leftMotor1 = new SparkMax(0, MotorType.kBrushless); //TODO add ids to sparkMaxs
        m_leftMotor2 = new SparkMax(1, MotorType.kBrushless);
        m_rightMotor1 = new SparkMax(2, MotorType.kBrushless);
        m_rightMotor2 = new SparkMax(3, MotorType.kBrushless);
        
        m_leftMotor1Config = new SparkMaxConfig();
        m_leftMotor2Config = new SparkMaxConfig();
        m_rightMotor1Config = new SparkMaxConfig();
        m_rightMotor2Config = new SparkMaxConfig();
    }

    public void extendClimberOutwards() {
        // code to extend the climber outwards using the motors and servos
    }

    public void extendClimberL1() {
        // code to extend the climber using the motors and servos
    }

    public void extendClimberL2AndL3() {
        // code to extend the climber using the motors and servos
    }
}