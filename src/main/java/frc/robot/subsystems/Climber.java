package frc.robot.subsystems;

import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
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
    private final  DigitalInput m_topLeftLimitSwitch = new DigitalInput(ClimberConstants.topLeftLimitSwitchID);
    private final  DigitalInput m_bottomLeftLimitSwitch = new DigitalInput(ClimberConstants.bottomLeftLimitSwitchID);
    private final  DigitalInput m_topRightLimitSwitch = new DigitalInput(ClimberConstants.topRightLimitSwitchID);
    private final  DigitalInput m_bottomRightLimitSwitch = new DigitalInput(ClimberConstants.bottomRightLimitSwitchID);
    
    private SparkMaxConfig m_climberMotorConfig;
    private SparkMaxConfig m_followMotorConfig;
    private double m_moveTo;
    private RelativeEncoder m_climberMotorRealtiveEncoder;
    

    public Climber() {
        // initialize motors, encoders, configs, etc. here
        m_climberMotor = new SparkMax(CANIdConstants.ClimberMotorID, MotorType.kBrushless); 
        m_climberMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();
        
        m_climberMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.motorStallLimit, ClimberConstants.motorFreeLimit);
        
        m_followMotorConfig.follow(CANIdConstants.ClimberMotorID, true); //Todo check if it needs to be inverted

        m_climberMotorRealtiveEncoder = m_climberMotor.getEncoder();

         m_climberMotorConfig.encoder
        .positionConversionFactor(1.0 / ClimberConstants.gearRatio);
    }

    public boolean climberHome() {
        if (m_bottomLeftLimitSwitch.get() && m_bottomRightLimitSwitch.get()) { //TO
            m_climberMotor.set(0);
            m_climberMotorRealtiveEncoder.setPosition(0);
            return true;
        }
         m_climberMotor.set(ClimberConstants.homeRPM);
            return false;
    }

    public boolean moveToPosition(double position) {
        if(Math.abs(position - m_climberMotorRealtiveEncoder.getPosition()) < ClimberConstants.positionTolerance_IN) {
                    stopClimber();
                    return true;
        }
        else if(position > m_climberMotorRealtiveEncoder.getPosition()) {
            m_climberMotor.set(ClimberConstants.motorRPM);
        }
        else if(position < m_climberMotorRealtiveEncoder.getPosition()) {
            m_climberMotor.set(-ClimberConstants.motorRPM);
        }
        
        if(m_topLeftLimitSwitch.get() || m_topRightLimitSwitch.get()) {
            m_climberMotor.stopMotor();
        }

         if(m_bottomLeftLimitSwitch.get() || m_bottomRightLimitSwitch.get()) {
            m_climberMotor.stopMotor();
        }
         return false;
    }

     public void extendHooks() {
        // code to extend the climber outwards using the motor.
       m_leftHookTopServo.setAngle(ClimberConstants.hookServoExtend);
       m_leftHookBottomServo.setAngle(ClimberConstants.hookServoExtend);
       m_rightHookTopServo.setAngle(ClimberConstants.hookServoExtend);
       m_rightHookBottomServo.setAngle(ClimberConstants.hookServoExtend);
    }

     public void retractHooks() {
       m_leftHookTopServo.setAngle(ClimberConstants.hookServoRetract);
       m_leftHookBottomServo.setAngle(ClimberConstants.hookServoRetract);
       m_rightHookTopServo.setAngle(ClimberConstants.hookServoRetract);
       m_rightHookBottomServo.setAngle(ClimberConstants.hookServoRetract);
       //TODO read encoder pulses to count howfar hooks are and retract them
       //TODO set inches to retract and read encoder pulses to know when to stop retracting //servo in servo out 
    }

    public void extendClimberL1() {
        // code to extend the climber outwards using the motor.
        m_climberMotorRealtiveEncoder.setPosition( ClimberConstants.extendClimberToL1_IN);
    }

    public void hookClimberL1() {
        m_climberMotor.set( ClimberConstants.l1climb_IN);
        // code to extend the climber using the motors and servos
    }

    public void hookClimberL2AndL3() {
          m_climberMotor.set( ClimberConstants.l2AndL3climb_IN);
          // code to extend the climber using the motors and servos
    }

    public void retractClimberL1() {
         m_climberMotor.set( ClimberConstants.homeRPM);
        // code to retract the climber using the motors and servos
    }

    public void retractClimberL2AndL3() {

        // code to retract the climber using the motors and servos
    }

    public void unlockRatchet() {
       m_leftRatchetServo.set(ClimberConstants.ratchetServoUnlock); 
       m_rightRatchetServo.set(ClimberConstants.ratchetServoUnlock); // code to unlock the ratchet when going down, lock when going up 
    }

    public void lockRatchet()  {
        m_leftRatchetServo.set(1 - ClimberConstants.ratchetServoUnlock); 
        m_rightRatchetServo.set(1 - ClimberConstants.ratchetServoUnlock); // code to unlock the ratchet when going down, lock when going up 
    }

    public void stopClimber(){
        m_climberMotor.stopMotor();
    }
    
}
