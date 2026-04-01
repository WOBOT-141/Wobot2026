package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.subsystems.SwerveModuleIO.ModuleIO;
import frc.robot.subsystems.SwerveModuleIO.ModuleIOInputsAutoLogged;

public class SwerveModule extends SubsystemBase {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final SysIdRoutine driveSysId;

    private double _xPos;
    private double _yPos;
    private double m_encoderOffset_rot;

    private int steerMotorID;
    private int driveMotorID;
    private double maxAmpsBL_amp = 0;
    private double maxAmpsBR_amp = 0;
    private double maxAmpsFL_amp = 0;
    private double maxAmpsFR_amp = 0;

    /**
     * Creates a new Wheel using AdvantageKit IO.
     */
    public SwerveModule(ModuleIO io, int steerId, int driveId, double absoluteOffset_deg, double xPosition, double yPosition) {
        this.io = io;
        this.steerMotorID = steerId;
        this.driveMotorID = driveId;
        this._xPos = xPosition;
        this._yPos = yPosition;
        this.m_encoderOffset_rot = absoluteOffset_deg;

        // 3. Give each module a unique name for the dashboard
        this.setName("Module" + driveId);

        driveSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        edu.wpi.first.units.Units.Volts.of(1.0).per(edu.wpi.first.units.Units.Second),
                        edu.wpi.first.units.Units.Volts.of(7.0),
                        edu.wpi.first.units.Units.Second.of(10.0)
                ),
                new SysIdRoutine.Mechanism(
                        volts -> {
                            setAngle(0);
                            setDriveVoltage(volts.in(edu.wpi.first.units.Units.Volts));
                        },
                        log -> {
                            log.motor("drive")
                                    .voltage(edu.wpi.first.units.Units.Volts.of(inputs.driveAppliedVolts))
                                    .linearPosition(edu.wpi.first.units.Units.Meters.of(inputs.drivePositionMeters))
                                    .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(inputs.driveVelocityMetersPerSec));
                        },
                        this // 4. PASS 'this' INSTEAD OF 'null'
                )
        );

        settoZero();
    }

    public double getEncoderOffset_deg() {
        return m_encoderOffset_rot;
    }

    public double get_xPos() {
        return _xPos;
    }

    public double get_yPos() {
        return _yPos;
    }

    public void setEncoderOffset(double m_encoderOffset_rot) {
        this.m_encoderOffset_rot = m_encoderOffset_rot;
    }

    public void set_xPos(double _xPos) {
        this._xPos = _xPos;
    }

    public void set_yPos(double _yPos) {
        this._yPos = _yPos;
    }

    public void setSpeed(double speed) {
        io.setDriveVelocity(speed);
        SmartDashboard.putNumber("The speed of commanding", speed);
    }

    public void setPos(double pos) {
        io.setDrivePosition(pos);
    }

    public void setAngle(double angle) {
        io.setSteerAngle(angle);
    }

    public void goToLockPosition() {
        if (_xPos * _yPos > 0) { // front right and back left
            setAngle(135);
        } else if (_xPos * _yPos < 0) { // front left and back right
            setAngle(225);
        }
    }

    public void cancelMovement() {
        io.stop();
    }

    public double getSpeed() {
        return inputs.driveVelocityMetersPerSec; // Make sure your original speed was in m/s!
    }

    public double getMaxAngularSpeedRadPerSec() {
        return Constants.DriveConstants.maxSpeedMetersPerSec / Constants.DriveConstants.driveBaseRadius;
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return Constants.DriveConstants.maxSpeedMetersPerSec;
    }

    public double getAbsoluteEncoderAngle_deg() {
        double startAngle_rot = inputs.steerAbsolutePositionRot;
        return (startAngle_rot - m_encoderOffset_rot) * 360;
    }

    public double getEncoderDistance() {
        return inputs.drivePositionInches;
    }

    public void resetEncoder() {
        // Only resets software states, doesn't zero absolute hardware
    }

    public SwerveModuleState bestAngle(SwerveModuleState wheel) {
        double currentAngle_deg = inputs.steerRelativePositionDeg;

        int totalRotations_rot = (int) (currentAngle_deg / 360);
        double wantedAngle_deg = wheel.angle.getDegrees() + 180;
        double complimentaryAngle_deg = (wantedAngle_deg + 180) % 360;

        double finalSmallestAngle_deg = 0;
        int direction = 0;

        double deltaCurrent_deg = (wantedAngle_deg + (360 * totalRotations_rot)) - currentAngle_deg;
        double deltaNext_deg = (wantedAngle_deg + (360 * (totalRotations_rot + 1))) - currentAngle_deg;
        double deltaPrevious_deg = (wantedAngle_deg + (360 * (totalRotations_rot - 1))) - currentAngle_deg;

        double deltaCurrentComp_deg = (complimentaryAngle_deg + (360 * totalRotations_rot)) - currentAngle_deg;
        double deltaNextComp_deg = (complimentaryAngle_deg + (360 * (totalRotations_rot + 1))) - currentAngle_deg;
        double deltaPreviousComp_deg = (complimentaryAngle_deg + (360 * (totalRotations_rot - 1))) - currentAngle_deg;

        double smallestAngle_deg = deltaPrevious_deg;
        if (Math.abs(smallestAngle_deg) > Math.abs(deltaCurrent_deg)) {
            smallestAngle_deg = deltaCurrent_deg;
        }
        if (Math.abs(smallestAngle_deg) > Math.abs(deltaNext_deg)) {
            smallestAngle_deg = deltaNext_deg;
        }

        double smallestAngleComp_deg = deltaPreviousComp_deg;
        if (Math.abs(smallestAngleComp_deg) > Math.abs(deltaCurrentComp_deg)) {
            smallestAngleComp_deg = deltaCurrentComp_deg;
        }
        if (Math.abs(smallestAngleComp_deg) > Math.abs(deltaNextComp_deg)) {
            smallestAngleComp_deg = deltaNextComp_deg;
        }

        if (Math.abs(smallestAngle_deg) <= Math.abs(smallestAngleComp_deg)) {
            finalSmallestAngle_deg = smallestAngle_deg;
            direction = -1;
        } else {
            finalSmallestAngle_deg = smallestAngleComp_deg;
            direction = 1;
        }

        return new SwerveModuleState(
                wheel.speedMetersPerSecond * direction,
                new Rotation2d(Units.degreesToRadians(currentAngle_deg + finalSmallestAngle_deg)));
    }

    // Call this from Drivetrain's periodic loop for each module
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + driveMotorID, inputs);

        SmartDashboard.putNumber("encoderDist", getEncoderDistance());
        SmartDashboard.putNumber("speeed", getSpeed());

        steerMotorPosDebug();
        absEncoderDebug();
        tractionMotorAMPDebug();
    }

    public void tractionMotorAMPDebug() {
        double outputCurrent_amp = inputs.driveCurrentAmps;
        switch (driveMotorID) {
            case CANIdConstants.backLeftDriveMotorID:
                if (outputCurrent_amp > maxAmpsBL_amp) {
                    maxAmpsBL_amp = outputCurrent_amp;
                }
                SmartDashboard.putNumber("Back Left Max Amps: ", maxAmpsBL_amp);
                break;
            case CANIdConstants.backRightDriveMotorID:
                if (outputCurrent_amp > maxAmpsBR_amp) {
                    maxAmpsBR_amp = outputCurrent_amp;
                }
                SmartDashboard.putNumber("Back Right Max Amps: ", maxAmpsBR_amp);
                break;
            case CANIdConstants.frontLeftDriveMotorID:
                if (outputCurrent_amp > maxAmpsFL_amp) {
                    maxAmpsFL_amp = outputCurrent_amp;
                }
                SmartDashboard.putNumber("Front Left Max Amps: ", maxAmpsFL_amp);
                break;
            case CANIdConstants.frontRightDriveMotorID:
                if (outputCurrent_amp > maxAmpsFR_amp) {
                    maxAmpsFR_amp = outputCurrent_amp;
                }
                SmartDashboard.putNumber("Front Right Max Amps: ", maxAmpsFR_amp);
                break;
        }
    }

    public void steerMotorPosDebug() {
        double currentAngle_deg = inputs.steerRelativePositionDeg;
        switch (steerMotorID) {
            case CANIdConstants.backLeftSteerMotorID:
                SmartDashboard.putNumber("Neo position Back Left", currentAngle_deg);
                break;
            case CANIdConstants.backRightSteerMotorID:
                SmartDashboard.putNumber("Neo position Back Right", currentAngle_deg);
                break;
            case CANIdConstants.frontLeftSteerMotorID:
                SmartDashboard.putNumber("Neo position Front Left", currentAngle_deg);
                break;
            case CANIdConstants.frontRightSteerMotorID:
                SmartDashboard.putNumber("Neo position Front Right", currentAngle_deg);
                break;
        }
    }

    public void absEncoderDebug() {
        double startAngle_deg = inputs.steerAbsolutePositionRot * 360.0;
        switch (steerMotorID) {
            case CANIdConstants.backLeftSteerMotorID:
                SmartDashboard.putNumber("Back Left Cancoder: ", startAngle_deg);
                break;
            case CANIdConstants.backRightSteerMotorID:
                SmartDashboard.putNumber("Back Right Cancoder: ", startAngle_deg);
                break;
            case CANIdConstants.frontLeftSteerMotorID:
                SmartDashboard.putNumber("Front Left Cancoder: ", startAngle_deg);
                break;
            case CANIdConstants.frontRightSteerMotorID:
                SmartDashboard.putNumber("Front Right Cancoder: ", startAngle_deg);
                break;
        }
    }

    public void settoZero() {
        double absoluteAngle = getAbsoluteEncoderAngle_deg();
        io.setSteerEncoderPosition(absoluteAngle);
    }

    public Translation2d getTranslation() {
        return new Translation2d(_xPos, _yPos);
    }

    public void runSetpoint(SwerveModuleState state) {
        setSpeed(state.speedMetersPerSecond);
        setAngle(state.angle.getDegrees());
    }

    // ---- SYSID HELPERS ----
    public void setDriveVoltage(double volts) {
        io.setDriveVoltage(volts);
    }

    public double getDrivePositionMeters() {
        return inputs.drivePositionMeters;
    }

    public double getDriveVelocityMetersPerSec() {
        return inputs.driveVelocityMetersPerSec;
    }

    public double getAppliedDriveVoltage() {
        return inputs.driveAppliedVolts;
    }

    public Command sysIdQuasistaticForward() {
        return driveSysId.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return driveSysId.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return driveSysId.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return driveSysId.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
