package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface IGyro {

    void ResetGyro();

    // sets the gyro offset in degrees
    void setGyroOffset(double offset);

    // gets the gyro offset in degrees
    double getGyroOffset();

    // gets the raw angle of the gyro (degrees rotated from start NOT 0-360)
    double getRawGyroAngle();

    // gets the real 0-360 angle of the robot relative to start
    double getRealGyroAngle();

    double getPitch();

    double getYaw();

    double getRoll();
}