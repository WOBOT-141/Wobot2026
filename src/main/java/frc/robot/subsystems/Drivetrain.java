package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.XYVector;
import frc.robot.Constants.WheelConstants;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] _wheels;
  private IGyro m_gyro;
  private double restAngle_deg;
  private boolean m_isTipping = false;
  private boolean m_climbing = false;
  private double m_tipMagnitude;
  private double m_tipDirection_deg;
  private double m_tipThreshold = 10;

  public Drivetrain(SwerveModule[] wheels, IGyro gyro) {
    _wheels = wheels;
    m_gyro = gyro;
  }

  public void setClimbing(boolean climb) {
    m_climbing = climb;
  }

  public void driveWithoutGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc) {
    driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, 0, 0, false);
  }

  public void driveWithoutGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc,
      double rotX, double rotY, boolean fromSticks) {
    if (m_isTipping && !m_climbing) {
      translationAngle_deg = m_tipDirection_deg;
      translationSpeed_perc = m_tipMagnitude / (m_tipThreshold * 2);
      rotationalSpeed_perc = 0;
      rotX = 0;
      rotY = 0;
    }

    // SmartDashboard.putNumber("translation Speed", translationSpeed_perc);
    // SmartDashboard.putNumber("translation Angle", translationAngle_deg);
    // SmartDashboard.putNumber("Rotational Speed", rotationalSpeed_perc);

    if (failedNaNcheck(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc))
      return;

    double rotationalCenterX = rotX;
    double rotationalCenterY = rotY;
    double[] calcWheelAngle_deg = new double[_wheels.length];
    double[] calcWheelSpeed_perc = new double[_wheels.length];
    double normalizingFactor = 1;

    if (translationAngle_deg == 0 && translationSpeed_perc == 0 && rotationalSpeed_perc == 0) {
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

      XYVector translationVector = getTranslationVector(translationSpeed_perc, translationAngle_deg);
      double rotationAngle = getRotationAngle(deltaX, deltaY, translationAngle_deg, rotationalSpeed_perc);
      XYVector rotationVector = getRotationVector(rotationAngle, rotationalSpeed_perc, deltaX, deltaY);
      XYVector resultXY = new XYVector(translationVector.X + rotationVector.X, translationVector.Y + rotationVector.Y);

      double resultSpeed_perc = Math.sqrt(Math.pow(resultXY.Y, 2) + Math.pow(resultXY.X, 2));

      double resultAngle_deg = getResultAngle(resultXY);

      calcWheelAngle_deg[i] = resultAngle_deg;
      calcWheelSpeed_perc[i] = resultSpeed_perc;

      // normalizing the vectors
      if (resultSpeed_perc > normalizingFactor) {
        normalizingFactor = resultSpeed_perc;
      }
    }

    for (int i = 0; i < _wheels.length; i++) {

      double translation = calcWheelSpeed_perc[i] / normalizingFactor;
      if (fromSticks)
        translation *= 1.5;

      var inputState = new SwerveModuleState(translation,
          Rotation2d.fromDegrees(calcWheelAngle_deg[i]));
      var outputState = _wheels[i].bestAngle(inputState);
      _wheels[i].setAngle(outputState.angle.getDegrees());
      _wheels[i].setSpeed(outputState.speedMetersPerSecond);
    }
  }

  public boolean failedNaNcheck(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc) {

    if (Double.isNaN(translationAngle_deg) || Double.isNaN(translationSpeed_perc) || Double.isNaN(rotationalSpeed_perc)) {
      System.out.println("ERROR: A Drivetrain commanded value is NaN!!!");
      System.out.printf("translationSpeed:%.3f  translationAngle:%.3f  rotationalSpeed:%.3f\n", translationSpeed_perc, translationAngle_deg, rotationalSpeed_perc);
      
      for (SwerveModule swerveModule : _wheels) {
        swerveModule.cancelMovement();
      }
      return true;
    }
    return false;
  }

  public void driveWithGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc,
      double rotX, double rotY, boolean fromSticks) {
    // Field-oriented adjustment
    translationAngle_deg -= m_gyro.getRealGyroAngle();
    driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, rotX, rotY, fromSticks);
  }

  public void driveWithGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc,
      double rotX, double rotY) {
    // Field-oriented adjustment
    translationAngle_deg -= m_gyro.getRealGyroAngle();
    driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, rotX, rotY, false);
  }

  public void driveWithGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc) {
    translationAngle_deg -= m_gyro.getRealGyroAngle();
    driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc);
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

  public XYVector getRotationVector(double rotationAngle_deg, double rotationalSpeed_perc, double deltaX,
      double deltaY) {
    double rotationalspeedY = Math.cos(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_perc);
    double rotationalspeedX = Math.sin(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_perc);
    if (deltaX == 0 && deltaY == 0) {
      rotationalspeedX = 0;
      rotationalspeedY = 0;
    }
    return new XYVector(rotationalspeedX, rotationalspeedY);
  }

  public XYVector getTranslationVector(double translationSpeed_perc, double translationAngle_deg) {
    double linDeltaY = Math.cos(Math.toRadians(translationAngle_deg)) * translationSpeed_perc;
    double linDeltaX = Math.sin(Math.toRadians(translationAngle_deg)) * translationSpeed_perc;
    return new XYVector(linDeltaX, linDeltaY);
  }

  public double getRotationAngle(double deltaX, double deltaY, double translationAngle_deg,
      double rotationalSpeed_perc) {
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
      // should be the translation angle
      alpha = translationAngle_deg;
    } else {
      // Otherwise use atan2 of delta X and delta Y to find angle
      alpha = Math.toDegrees(Math.atan2(deltaX, deltaY));
    }

    if (rotationalSpeed_perc > 0) {
      // If rotating clockwise add 90 to alpha to get angle
      RotationAngle_deg = alpha + 90;
    } else if (rotationalSpeed_perc < 0) {
      // If rotating counter clockwise subtract 90 from alpha to get angle
      RotationAngle_deg = alpha - 90;
    }

    return RotationAngle_deg;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    driveWithoutGyro(0, 0, 0);
  }

  public void lockWheels() {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].goToLockPosition();
    }
  }

  public SwerveModule[] getWheels() {
    return _wheels;
  }

  public void sysIdQuasistaticForward() {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].sysIdQuasistaticForward();
    }
  }

  public void sysIdQuasistaticReverse() {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].sysIdQuasistaticReverse();
    }
  }

  public void sysIdDynamicForward() {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].sysIdDynamicForward();
    }
  }

  public void sysIdDynamicReverse() {
    for (int i = 0; i < _wheels.length; i++) {
      _wheels[i].sysIdDynamicReverse();
    }
  }
}
