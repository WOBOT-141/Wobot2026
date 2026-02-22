// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NavxGyro extends SubsystemBase implements IGyro {

  private final AHRS gyro;

  private double gyroOffset = 0;

  // Dashboard state caching
  private Boolean lastConnected = null;
  private Boolean lastCalibrated = null;

  /** Creates a new NavX Gyro */
  public NavxGyro() {
    gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);
  }

  /** Returns true if the navX is connected */
  public boolean isConnected() {
    return gyro.isConnected();
  }

  /** Returns true while the navX is calibrating */
  public boolean isCalibrating() {
    return gyro.isCalibrating();
  }

  /** Returns true only when the navX is connected AND finished calibrating */
  public boolean isCalibrated() {
    return isConnected() && !isCalibrating();
  }

  @Override
  public void ResetGyro() {
    gyro.reset();
    gyroOffset = 0;
  }

  // sets the gyro offset in degrees
  @Override
  public void setGyroOffset(double offset) {
    gyroOffset = offset;
  }

  // gets the gyro offset in degrees
  @Override
  public double getGyroOffset() {
    return gyroOffset;
  }

  // gets the raw angle of the gyro (degrees rotated from start NOT 0-360)
  @Override
  public double getRawGyroAngle() {
    return gyro.getAngle();
  }

  // gets the real 0-360 angle of the robot relative to start
  @Override
  public double getRealGyroAngle() {
    double angle = (getRawGyroAngle() + gyroOffset) % 360;

    if (angle < 0) {
      angle += 360;
    }

    return angle;
  }

  @Override
  public double getPitch() {
    return gyro.getPitch();
  }

  @Override
  public double getYaw() {
    return gyro.getYaw();
  }

  @Override
  public double getRoll() {
    return gyro.getRoll();
  }

  @Override
  public Rotation2d getRotationAs2d() {
    SmartDashboard.putNumber("NAVXGYROANG", getYaw());
    SmartDashboard.putNumber("NAVXROT2DVALUE", new Rotation2d(Units.degreesToRadians(getYaw())).getDegrees());

    return new Rotation2d(Units.degreesToRadians(getYaw()));
  }

  /**
   * Finds best angle of rotation to reach a given angle 0–360
   *
   * @param wantedAngle angle wanted to rotate to (degrees)
   * @return the smallest possible angle to reach wanted angle
   */
  public double bestAngle(double wantedAngle) {
    double currentAngle = getRawGyroAngle();
    int totalRotations = (int) (currentAngle / 360);

    double deltaCurrent = (wantedAngle + (360 * totalRotations)) - currentAngle;
    double deltaNext = (wantedAngle + (360 * (totalRotations + 1))) - currentAngle;
    double deltaPrevious = (wantedAngle + (360 * (totalRotations - 1))) - currentAngle;

    double smallestAngle = deltaPrevious;

    if (Math.abs(smallestAngle) > Math.abs(deltaCurrent)) {
      smallestAngle = deltaCurrent;
    }
    if (Math.abs(smallestAngle) > Math.abs(deltaNext)) {
      smallestAngle = deltaNext;
    }

    return smallestAngle + currentAngle; // left as-is per your TODO
  }

  @Override
  public void periodic() {
    // Always-changing values
    SmartDashboard.putNumber("Gyro", getRawGyroAngle());
    SmartDashboard.putNumber("Gyro - ABS", getRealGyroAngle());

    // Connected status (only update on change)
    boolean connected = isConnected();
    if (lastConnected == null || connected != lastConnected) {
      SmartDashboard.putBoolean("NavX Is Connected", connected);
      lastConnected = connected;
    }

    // Calibrated status (only update on change)
    boolean calibrated = isCalibrated();
    if (lastCalibrated == null || calibrated != lastCalibrated) {
      SmartDashboard.putBoolean("NavX Is Calibrated", calibrated);
      lastCalibrated = calibrated;
    }
  }
}
