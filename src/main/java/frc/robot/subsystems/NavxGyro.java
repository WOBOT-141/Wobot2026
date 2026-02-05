// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavxGyro extends SubsystemBase implements IGyro {
  private AHRS gyro;
  double gyroOffset = 0;

  /** Creates a new Gyro. */
  public NavxGyro() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
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
  public double getPitch(){
    return gyro.getPitch();
  }

  @Override
  public double getYaw(){
    return gyro.getYaw();
  }

  @Override
  public double getRoll(){
    return gyro.getRoll();
  }

  /**
   * finds best angle of rotation to reach a given angle 0-360
   * 
   * @param wantedRotatation angle wanted to rotate (degrees)
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
    return smallestAngle + currentAngle; // TODO I don't think we want to add currentAngle here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", getRawGyroAngle());
    SmartDashboard.putNumber("Gyro - ABS", getRealGyroAngle());
    // System.out.println(getRealGyroAngle());
    // System.out.printf("Angle: %.2f Yaw: %.2f\n", gyro.getAngle(), gyro.getYaw());
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("getAngle'");
  }

  @Override
  public void zeroYaw() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("zeroYaw'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("reset'");
  }
}