package frc.robot.subsystems;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.climberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;




public class Climber extends SubsystemBase {
    private final SparkMax m_climberMotor;
    private Servo m_leftHookTopServo;
    private Servo m_leftHookBottomServo; // HS-322HD Heavy Duty Resin Gear 24T Analog Sport Servos 
    private Servo m_leftRatchetServo; // HS-322HD Heavy Duty Resin Gear 24T Analog Sport Servos
    private Servo m_rightHookTopServo;
    private Servo m_rightHookBottomServo;
    private Servo m_rightRatchetServo;
    private SparkMaxConfig m_climberMotorConfig;
    private SparkMaxConfig m_followMotorConfig;
    private double m_moveTo;
    private RelativeEncoder m_climberMotorRealtiveEncoder;
    

    public Climber() {
        // initialize motors, encoders, configs, etc. here
        m_climberMotor = new SparkMax(CANIdConstants.ClimberMotorID, MotorType.kBrushless); //TODO add ids to sparkMaxs addto constants 
        
        m_climberMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();
        
        m_climberMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(climberConstants.motorStallLimit, climberConstants.motorFreeLimit);
        
        m_followMotorConfig.follow(CANIdConstants.ClimberMotorID, true); //Todo check if it needs to be inverted

        m_climberMotorRealtiveEncoder = m_climberMotor.getEncoder();

         m_climberMotorConfig.encoder
        .positionConversionFactor(1.0 / climberConstants.gearRatio);
    }

    public boolean climberHome() {
        boolean LimitSwitch = false; //TODO add limit switch to detect when at home position
        if (LimitSwitch) {
            m_climberMotor.set(0);
            m_climberMotorRealtiveEncoder.setPosition(0);
            return true;
        }
         m_climberMotor.set(climberConstants.homeRPM);
            return false;
    }

    public void moveToPosition(double position) {
        
        
       
       
        //TODO move to a position 
        //TODO check if position is within limits
        //TODO read encoder position to know if its where supposed tobe
        //TODO check motor position to see if at position and stop motor if at position 
        //TODO return true if at position false if not at position tolernce of inches to see if true 
    }

     public void extendHooks() {
        // code to extend the climber outwards using the motor.
       m_leftHookTopServo.setAngle(climberConstants.hookServoOn);
       m_leftHookBottomServo.setAngle(climberConstants.hookServoOn);
       m_rightHookTopServo.setAngle(climberConstants.hookServoOn);
       m_rightHookBottomServo.setAngle(climberConstants.hookServoOn);
       //TODO set digital outputs make one command to tell them all on 

    }

     public void retractHooks() {
        // code to extend the climber outwards using the motor.
       m_leftHookTopServo.setAngle(climberConstants.hookServoOff);
       m_leftHookBottomServo.setAngle(climberConstants.hookServoOff);
       m_rightHookTopServo.setAngle(climberConstants.hookServoOff);
       m_rightHookBottomServo.set(0);
       //TODO read encoder pulses to count howfar hooks are and retract them
       //TODO set inches to retract and read encoder pulses to know when to stop retracting
    }

    

    public void extendClimberL1() {
        // code to extend the climber outwards using the motor.
        m_climberMotorRealtiveEncoder.setPosition( climberConstants.extendClimberToL1_IN);
    }

    public void hookClimberL1() {
        m_climberMotor.set( climberConstants.l1climb_IN);
        // code to extend the climber using the motors and servos
    }

    public void hookClimberL2AndL3() {
          m_climberMotor.set( climberConstants.l2AndL3climb_IN);
          // code to extend the climber using the motors and servos
    }

    public void retractClimberL1() {
         m_climberMotor.set( climberConstants.homeRPM);
        // code to retract the climber using the motors and servos
    }

    public void retractClimberL2AndL3() {

        // code to retract the climber using the motors and servos
    }

    public void ratchetUnlock() {
       m_leftRatchetServo.set(climberConstants.ratchetServoUnlock);
       m_rightRatchetServo.set(climberConstants.ratchetServoUnlock);
    }


        
   

}