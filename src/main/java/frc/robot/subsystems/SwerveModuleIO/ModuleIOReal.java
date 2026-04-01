package frc.robot.subsystems.SwerveModuleIO;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WheelConstants;

public class ModuleIOReal implements ModuleIO {

    private final SparkMax _driveMotor;
    private final SparkMax _steerMotor;
    private final CANcoder _cancoder;
    
    private final RelativeEncoder _driveEncoder;
    private final RelativeEncoder _steerEncoder;
    
    private final SparkClosedLoopController _drivePID;
    private final SparkClosedLoopController _steerPID;

    public ModuleIOReal(int steerId, int driveId, int cancoderId, double driveRotationsPerInch) {
        
        _cancoder = new CANcoder(cancoderId);
        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        _cancoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.withAbsoluteSensorDiscontinuityPoint(1);
        _cancoder.getConfigurator().apply(encoderConfig);

        _driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WheelConstants.driveCurrentLimit_amp, WheelConstants.driveCurrentLimit_amp);
        driveConfig.encoder
            .positionConversionFactor(1.0 / driveRotationsPerInch)
            .velocityConversionFactor((1.0 / driveRotationsPerInch) / 60.0);

        FeedForwardConfig driveFF = new FeedForwardConfig();
        driveFF.kV(WheelConstants.DriveFFKv);

        driveConfig.closedLoop
            .p(WheelConstants.DriveP)
            .i(WheelConstants.DriveI)
            .d(WheelConstants.DriveD)
            .apply(driveFF);

        _driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _driveEncoder = _driveMotor.getEncoder();
        _drivePID = _driveMotor.getClosedLoopController();

        _steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        
        steerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WheelConstants.steerCurrentLimit_amp, WheelConstants.steerCurrentLimit_amp);
        steerConfig.encoder
            .positionConversionFactor(WheelConstants.steerDegPerMotorRotation_deg);
        steerConfig.closedLoop
            .pid(WheelConstants.SteerP, WheelConstants.SteerI, WheelConstants.DriveD, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(5, ClosedLoopSlot.kSlot0);

        _steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _steerEncoder = _steerMotor.getEncoder();
        _steerPID = _steerMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Drive Inputs
        inputs.drivePositionInches = _driveEncoder.getPosition();
        inputs.drivePositionMeters = Units.inchesToMeters(_driveEncoder.getPosition());
        inputs.driveVelocityMetersPerSec = Units.inchesToMeters(_driveEncoder.getVelocity());
        inputs.driveAppliedVolts = _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = _driveMotor.getOutputCurrent();

        // Steer Inputs
        inputs.steerAbsolutePositionRot = _cancoder.getAbsolutePosition().getValueAsDouble();
        inputs.steerRelativePositionDeg = _steerEncoder.getPosition();
        inputs.steerAppliedVolts = _steerMotor.getAppliedOutput() * _steerMotor.getBusVoltage();
        inputs.steerCurrentAmps = _steerMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVelocity(double speed) {
        _drivePID.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setDrivePosition(double pos) {
        _drivePID.setSetpoint(pos, ControlType.kPosition);
    }

    @Override
    public void setSteerAngle(double angleDeg) {
        _steerPID.setSetpoint(angleDeg, ControlType.kPosition);
    }

    @Override
    public void setDriveVoltage(double volts) {
        _driveMotor.setVoltage(volts);
    }

    @Override
    public void setSteerEncoderPosition(double positionDeg) {
        _steerEncoder.setPosition(positionDeg);
    }

    @Override
    public void stop() {
        _driveMotor.stopMotor();
        _steerMotor.stopMotor();
    }
}