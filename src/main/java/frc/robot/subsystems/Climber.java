package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.ControllerOneConstants.climberConstants;
import frc.robot.subsystems.SwerveModule.WheelConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;



public class Climber extends SubsystemBase {
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;
    private Servo m_leftHookTopServo;
    private Servo m_leftHookBottomServo; // HS-322HD Heavy Duty Resin Gear 24T Analog Sport Servos 
    private Servo m_leftRatchetServo; // HS-322HD Heavy Duty Resin Gear 24T Analog Sport Servos
    private Servo m_rightHookTopServo;
    private Servo m_rightHookBottomServo;
    private Servo m_rightRatchetServo;
    private SparkMaxConfig m_leftMotorConfig;
    private SparkMaxConfig m_rightMotorConfig;
    private double m_moveTo;
    private RelativeEncoder m_leftMotorRealtiveEncoder;
    private RelativeEncoder m_rightMotorRealtiveEncoder;
    

    public Climber() {
        // initialize motors, encoders, configs, etc. here
        m_leftMotor = new SparkMax(CANIdConstants.leftMotorID, MotorType.kBrushless); //TODO add ids to sparkMaxs addto constants 
        m_rightMotor = new SparkMax(CANIdConstants.rightMotorID, MotorType.kBrushless);
        
        m_leftMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig = new SparkMaxConfig();
        
        m_leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);

        m_rightMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60,60);

        m_leftMotorRealtiveEncoder = m_leftMotor.getEncoder();
        m_rightMotorRealtiveEncoder = m_rightMotor.getEncoder();

            m_leftMotorConfig.encoder
        .positionConversionFactor(1.0 / climberConstants.gearRatio);
    }

    public void climberToHome(){
        m_leftMotorRealtiveEncoder.setPosition(climberConstants.homePosition_IN);
    }

        public void extendHooks()
        // code to extend the climber outwards using the motor.
       m_leftHookTopServo.setAngle(climberConstants.hookServoOn);
       m_leftHookBottomServo.setAngle(climberConstants.hookServoOn);
    }

    public void extendClimberL1() {
        // code to extend the climber outwards using the motor.
        m_leftMotorRealtiveEncoder.setPosition( climberConstants.extendClimberToL1_IN);
    }

    public void hookClimberL1() {
        m_leftMotorRealtiveEncoder.setPosition( climberConstants.l1climb_IN);
        // code to extend the climber using the motors and servos
    }

    public void hookClimberL2AndL3() {
         m_leftMotorRealtiveEncoder.setPosition( climberConstants.l2AndL3climb_IN);
          // code to extend the climber using the motors and servos
    }

    public void retractClimberL1() {

        // code to retract the climber using the motors and servos
    }

    public void retractClimberL2AndL3() {

        // code to retract the climber using the motors and servos
    }

    public void ratchetLockAndUnlock() {
        // code to stop the climber using the motors and servos
    }


        
   

}