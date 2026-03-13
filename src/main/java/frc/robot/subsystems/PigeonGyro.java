// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonGyro extends SubsystemBase implements IGyro {
  /** Creates a new PigeonGyro. */
  private final int pigeonDeviceID = 13;
  Pigeon2 pigeon2;
  double gyroOffset = 0.0;

  public PigeonGyro() {
    pigeon2 = new Pigeon2(pigeonDeviceID, "rio");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PigeonGyro", getRawGyroAngle());
  }

  public void setGyroOffset(double offset) {
    gyroOffset = offset;
  }

  public double getGyroOffset() {
    return gyroOffset;
  }

  public double getRawGyroAngle() {
    return pigeon2.getYaw().getValue().in(Degrees)*-1;
  }

  public double getRealGyroAngle() {
    
    double angle = (getRawGyroAngle() + gyroOffset) % 360;
    
    if (angle < 0) {
      angle += 360;
    }

    return angle;
  }

  @Override
  public void ResetGyro() {
    pigeon2.reset();
    gyroOffset = 0.0;
  }

  @Override
  public double getPitch() {
    return pigeon2.getPitch().getValue().in(Degrees)*-1;
  }

  @Override
  public double getYaw() {
    return pigeon2.getYaw().getValue().in(Degrees)*-1;
  }

  @Override
  public double getRoll() {
    return pigeon2.getRoll().getValue().in(Degrees)*-1;
  }

  @Override
  public Rotation2d getRotationAs2d() {
    return Rotation2d.fromDegrees(getRawGyroAngle());
  }

}
