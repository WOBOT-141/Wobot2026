package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.XYVector;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private Wheel[] _wheels;
  private double restAngle_deg = 0;
  private boolean isTipping = false;
  private boolean climbing = false;
  private double tipMagnitude;
  private double tipDirection;
  private double tipThreshold = 10;

  public Drive(Wheel[] wheels, IGyro gyro) {
    _wheels = wheels;
  }

  public void setClimbing(boolean climb){
    climbing = climb;
  }

  public void driveSwerve(double linearAngle_deg, double linearSpeed_mps, double rotationalSpeed_dps) {
    driveSwerve(linearAngle_deg, linearSpeed_mps, rotationalSpeed_dps, 0, 0);
  }

  public void driveSwerve(double linearAngle_deg, double linearSpeed_mps, double rotationalSpeed_dps, double rotX, double rotY) {
    if(isTipping && !climbing){
      linearAngle_deg = tipDirection;
      linearSpeed_mps = tipMagnitude / (tipThreshold*2);
      rotationalSpeed_dps = 0;
      rotX = 0;
      rotY = 0;
    }
    SmartDashboard.putNumber("Linear Speed", linearSpeed_mps);
    SmartDashboard.putNumber("Linear Angle", linearAngle_deg);
    SmartDashboard.putNumber("Rotational Speed", rotationalSpeed_dps);

    double rotationalCenterX = rotX;
    double rotationalCenterY = rotY;
    double[] calcWheelAngle_deg = new double[_wheels.length];
    double[] calcWheelSpeed_mps = new double[_wheels.length];
    double normalizingFactor = 1;

    if (linearAngle_deg == 0 && linearSpeed_mps == 0 && rotationalSpeed_dps == 0) {
      for (int i = 0; i < _wheels.length; ++i) {
        _wheels[i].cancelMovement();
      }
      return;
    }

    // Calculate each wheel's angle and speed
    for (int i = 0; i < _wheels.length; ++i) {
      double wheelX = _wheels[i].get_xPos();
      double wheelY = _wheels[i].get_yPos();

      double deltaX = wheelX - rotationalCenterX;
      double deltaY = wheelY - rotationalCenterY;

      XYVector linearVector = getLinearVector(linearSpeed_mps, linearAngle_deg);
      double rotationAngle = getRotationAngle(deltaX, deltaY, linearAngle_deg, rotationalSpeed_dps);
      XYVector rotationVector = getRotationVector(rotationAngle, rotationalSpeed_dps, deltaX, deltaY);
      XYVector resultXY = new XYVector(linearVector.X + rotationVector.X, linearVector.Y + rotationVector.Y);

      double resultSpeed_mps = Math.sqrt(Math.pow(resultXY.Y, 2) + Math.pow(resultXY.X, 2));

      double resultAngle_deg = getResultAngle(resultXY);

      calcWheelAngle_deg[i] = resultAngle_deg;
      calcWheelSpeed_mps[i] = resultSpeed_mps;

      // normalizing the vectors
      if (resultSpeed_mps > normalizingFactor) {
        normalizingFactor = resultSpeed_mps;
      }
    }

    for (int i = 0; i < _wheels.length; i++) {
      var inputState = new SwerveModuleState(DriveConstants.topSpeed_mps * (calcWheelSpeed_mps[i] / normalizingFactor),
          Rotation2d.fromDegrees(calcWheelAngle_deg[i]));
      var outputState = _wheels[i].bestAngle(inputState);
      // _wheels[i].setAngle(outputState.angle.getDegrees());
      // _wheels[i].setSpeed(outputState.speedMetersPerSecond);
      // if (i == 3) {
      _wheels[i].setAngle(outputState.angle.getDegrees());
      _wheels[i].setSpeed(outputState.speedMetersPerSecond);
      // System.out.println("angle " + calcWheelAngle[i]);
      // }
      // System.out.println(_wheels[i]);
      // System.out.println("angle " + calcWheelAngle[i]);
      // System.out.println("speed " + calcWheelSpeed[i] / normalizingFactor);
    }
  }

  // Temp command for testing
  public void goDistance(double pos) {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].setPos(pos);
    }
  }

  public double getResultAngle(XYVector resultXY) {
    double resultAngle_deg = Math.toDegrees(Math.atan2(resultXY.Y, resultXY.X));
    resultAngle_deg = -(90 - resultAngle_deg);

    if (resultXY.Y == 0 && resultXY.X == 0) {
      resultAngle_deg = restAngle_deg;
    } else {
      restAngle_deg = resultAngle_deg;
    }
    return resultAngle_deg;
  }

  public XYVector getRotationVector(double rotationAngle_deg, double rotationalSpeed_dps, double deltaX, double deltaY) {
    double rotationalspeedY = Math.cos(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_dps);
    double rotationalspeedX = Math.sin(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_dps);
    if (deltaX == 0 && deltaY == 0) {
      rotationalspeedX = 0;
      rotationalspeedY = 0;
    }
    return new XYVector(rotationalspeedX, rotationalspeedY);
  }

  public XYVector getLinearVector(double linearSpeed_mps, double linearAngle_deg) {
    double linDeltaY = Math.cos(Math.toRadians(linearAngle_deg)) * linearSpeed_mps;
    double linDeltaX = Math.sin(Math.toRadians(linearAngle_deg)) * linearSpeed_mps;
    return new XYVector(linDeltaX, linDeltaY);
  }

  public double getRotationAngle(double deltaX, double deltaY, double linearAngle_deg, double rotationalSpeed_dps) {
    // alpha is the angle of the vector to the wheel location
    // with respect forward and in degrees
    double alpha = 0;
    double RotationAngle_deg = 0.0;

    if (deltaY == 0 && deltaX > 0) {
      // If the wheel is aligned with the center of rotation on the Y axis and
      // positive on the X axis alpha = 90
      alpha = 90;
    } else if (deltaY == 0 && deltaX < 0) {
      // If the wheel is aligned with the center of rotation on the Y axis and
      // negative on the X axis alpha = -90
      alpha = -90;
    } else if (deltaY == 0 && deltaX == 0) {
      // If the wheel is aligned with the center of rotation on both axis the alpha
      // should be the linear angle
      alpha = linearAngle_deg;
    } else {
      // Otherwise use atan2 of delta X and delta Y to find angle
      alpha = Math.toDegrees(Math.atan2(deltaX, deltaY));
    }

    if (rotationalSpeed_dps > 0) {
      // If rotating clockwise add 90 to alpha to get angle
      RotationAngle_deg = alpha + 90;
    } else if (rotationalSpeed_dps < 0) {
      // If rotating counter clockwise subtract 90 from alpha to get angle
      RotationAngle_deg = alpha - 90;
    }

    return RotationAngle_deg;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void lockWheels() {
    // Implement wheel-locking logic if needed
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].setAngle((i + 45) * 90);
    }
  }

  public Wheel[] getWheels() {
    return _wheels;
  }
}