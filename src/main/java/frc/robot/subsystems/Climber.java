package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ControllerOneConstants.climberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Climber extends SubsystemBase {
    private final SparkMax m_leftMotor1;
    private final SparkMax m_leftMotor2;
    private final SparkMax m_rightMotor1;
    private final SparkMax m_rightMotor2;
    private int leftServo1;
    private int leftServo2; // couldnt find much on servos
    private int rightServo1;
    private int rightServo2;
    private SparkMaxConfig m_leftMotor1Config;
    private SparkMaxConfig m_leftMotor2Config; 
    private SparkMaxConfig m_rightMotor1Config;
    private SparkMaxConfig m_rightMotor2Config;
    private double moveTo;
    

    public Climber() {
        // initialize motors, encoders, configs, etc. here
        m_leftMotor1 = new SparkMax(CANIdConstants.leftMotor1ID, MotorType.kBrushless); //TODO add ids to sparkMaxs addto constants 
        m_leftMotor2 = new SparkMax(CANIdConstants.leftMotor2ID, MotorType.kBrushless);
        m_rightMotor1 = new SparkMax(CANIdConstants.rightMotor1ID, MotorType.kBrushless);
        m_rightMotor2 = new SparkMax(CANIdConstants.rightMotor2ID, MotorType.kBrushless);
        
        m_leftMotor1Config = new SparkMaxConfig();
        m_leftMotor2Config = new SparkMaxConfig();
        m_rightMotor1Config = new SparkMaxConfig();
        m_rightMotor2Config = new SparkMaxConfig();
        
        m_leftMotor1Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);
        
        m_leftMotor2Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);

        m_rightMotor1Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);

        m_rightMotor2Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);
    }

    public void goToHome(){
        
    }


    public void extendClimber() {
        // code to extend the climber outwards using the motors and servos
    }

    public void hookClimberL1() {
        double i = climberConstants.l1climb_IN; 


        // code to extend the climber using the motors and servos
    }
    public void hookClimberL2AndL3() {
         
        // code to extend the climber using the motors and servos
    }
}