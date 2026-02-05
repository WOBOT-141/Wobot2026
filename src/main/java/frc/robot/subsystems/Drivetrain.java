package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.XYVector;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] _wheels;
  private double restAngle = 0;
  private boolean isTipping = false;
  private boolean climbing = false;
  private double tipMagnitude;
  private double tipDirection;
  private double tipThreshold = 10;

  public Drivetrain(SwerveModule[] wheels, IGyro gyro) {
    _wheels = wheels;
  }

  public void setClimbing(boolean climb){
    climbing = climb;
  }

  public void driveSwerve(double linearAngle, double linearSpeed, double rotationalSpeed) {
    driveSwerve(linearAngle, linearSpeed, rotationalSpeed, 0, 0);
  }

  public void driveSwerve(double linearAngle, double linearSpeed, double rotationalSpeed, double rotX, double rotY) {
    if(isTipping && !climbing){
      linearAngle = tipDirection;
      linearSpeed = tipMagnitude / (tipThreshold*2);
      rotationalSpeed = 0;
      rotX = 0;
      rotY = 0;
    }
    SmartDashboard.putNumber("Linear Speed", linearSpeed);
    SmartDashboard.putNumber("Linear Angle", linearAngle);
    SmartDashboard.putNumber("Rotational Speed", rotationalSpeed);

    double rotationalCenterX = rotX;
    double rotationalCenterY = rotY;
    double[] calcWheelAngle = new double[_wheels.length];
    double[] calcWheelSpeed = new double[_wheels.length];
    double normalizingFactor = 1;

    if (linearAngle == 0 && linearSpeed == 0 && rotationalSpeed == 0) {
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

      XYVector linearVector = getLinearVector(linearSpeed, linearAngle);
      double rotationAngle = getRotationAngle(deltaX, deltaY, linearAngle, rotationalSpeed);
      XYVector rotationVector = getRotationVector(rotationAngle, rotationalSpeed, deltaX, deltaY);
      XYVector resultXY = new XYVector(linearVector.X + rotationVector.X, linearVector.Y + rotationVector.Y);

      double resultSpeed = Math.sqrt(Math.pow(resultXY.Y, 2) + Math.pow(resultXY.X, 2));

      double resultAngle = getResultAngle(resultXY);

      calcWheelAngle[i] = resultAngle;
      calcWheelSpeed[i] = resultSpeed;

      // normalizing the vectors
      if (resultSpeed > normalizingFactor) {
        normalizingFactor = resultSpeed;
      }
    }

    for (int i = 0; i < _wheels.length; i++) {
      var inputState = new SwerveModuleState(DriveConstants.topSpeed * (calcWheelSpeed[i] / normalizingFactor),
          Rotation2d.fromDegrees(calcWheelAngle[i]));
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
    double resultAngle = Math.toDegrees(Math.atan2(resultXY.Y, resultXY.X));
    resultAngle = -(90 - resultAngle);

    if (resultXY.Y == 0 && resultXY.X == 0) {
      resultAngle = restAngle;
    } else {
      restAngle = resultAngle;
    }
    return resultAngle;
  }

  public XYVector getRotationVector(double rotationAngle, double rotationalSpeed, double deltaX, double deltaY) {
    double rotationalspeedY = Math.cos(Math.toRadians(rotationAngle)) * Math.abs(rotationalSpeed);
    double rotationalspeedX = Math.sin(Math.toRadians(rotationAngle)) * Math.abs(rotationalSpeed);
    if (deltaX == 0 && deltaY == 0) {
      rotationalspeedX = 0;
      rotationalspeedY = 0;
    }
    return new XYVector(rotationalspeedX, rotationalspeedY);
  }

  public XYVector getLinearVector(double linearSpeed, double linearAngle) {
    double linDeltaY = Math.cos(Math.toRadians(linearAngle)) * linearSpeed;
    double linDeltaX = Math.sin(Math.toRadians(linearAngle)) * linearSpeed;
    return new XYVector(linDeltaX, linDeltaY);
  }

  public double getRotationAngle(double deltaX, double deltaY, double linearAngle, double rotationalSpeed) {
    // alpha is the angle of the vector to the wheel location
    // with respect forward and in degrees
    double alpha = 0;
    double RotationAngle = 0.0;

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
      alpha = linearAngle;
    } else {
      // Otherwise use atan2 of delta X and delta Y to find angle
      alpha = Math.toDegrees(Math.atan2(deltaX, deltaY));
    }

    if (rotationalSpeed > 0) {
      // If rotating clockwise add 90 to alpha to get angle
      RotationAngle = alpha + 90;
    } else if (rotationalSpeed < 0) {
      // If rotating counter clockwise subtract 90 from alpha to get angle
      RotationAngle = alpha - 90;
    }

    return RotationAngle;
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

  public SwerveModule[] getWheels() {
    return _wheels;
  }
}