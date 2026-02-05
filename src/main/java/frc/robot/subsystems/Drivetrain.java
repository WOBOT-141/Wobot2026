
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/Drivetrain.java
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
=======
    private final Wheel[] wheels;
    private final SwerveDriveKinematics kinematics;

    private boolean isTipping = false;
    private boolean climbing = false;
    private double tipMagnitude;
    private double tipDirection;
    private final double tipThreshold = 10; // degrees
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Drive.java

    /**
     * Constructor – only takes the array of Wheel modules
     */
    public Drive(Wheel[] wheels) {
        this.wheels = wheels;

<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/Drivetrain.java
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
=======
        // Build kinematics using wheel positions
        // Your coordinate system: right = +x, forward = +y
        // WPILib: forward = +x, left = +y → we convert accordingly
        Translation2d[] modulePositions = new Translation2d[wheels.length];
        for (int i = 0; i < wheels.length; i++) {
            double wpilibX = wheels[i].get_yPos();     // forward → x
            double wpilibY = -wheels[i].get_xPos();    // right → -y (left positive)
            modulePositions[i] = new Translation2d(wpilibX, wpilibY);
        }

        this.kinematics = new SwerveDriveKinematics(modulePositions);
    }

    public void setClimbing(boolean climb) {
        climbing = climb;
    }

    public void driveSwerve(double linearAngle_deg, double linearSpeed_mps, double rotationalSpeed_dps) {
        driveSwerve(linearAngle_deg, linearSpeed_mps, rotationalSpeed_dps, 0, 0);
    }

    public void driveSwerve(double linearAngle_deg, double linearSpeed_mps,
                             double rotationalSpeed_dps, double rotX, double rotY) {

        if (isTipping && !climbing) {
            linearAngle_deg = tipDirection;
            linearSpeed_mps = tipMagnitude / (tipThreshold * 2);
            rotationalSpeed_dps = 0;
            rotX = 0;
            rotY = 0;
        }

        SmartDashboard.putNumber("Linear Speed (m/s)", linearSpeed_mps);
        SmartDashboard.putNumber("Linear Angle (deg)", linearAngle_deg);
        SmartDashboard.putNumber("Rotational Speed (deg/s)", rotationalSpeed_dps);

        if (Math.abs(linearSpeed_mps) < 1e-6 && Math.abs(rotationalSpeed_dps) < 1e-6) {
            for (Wheel wheel : wheels) {
                wheel.cancelMovement();
            }
            return;
        }

        // Convert inputs to chassis speeds (robot coordinate frame)
        double vx = linearSpeed_mps * Math.sin(Math.toRadians(linearAngle_deg));   // right
        double vy = linearSpeed_mps * Math.cos(Math.toRadians(linearAngle_deg));   // forward
        double omegaRad = Math.toRadians(rotationalSpeed_dps);

        // Adjust for rotation center offset (if rotX, rotY are used)
        vx += omegaRad * rotY;   // right component from rotation
        vy -= omegaRad * rotX;   // forward component from rotation

        // WPILib coordinate system: +x forward, +y left, +omega CCW
        double wpilibVx = vy;             // forward
        double wpilibVy = -vx;            // left = -right
        double wpilibOmega = omegaRad;    // CCW positive

        ChassisSpeeds speeds = new ChassisSpeeds(wpilibVx, wpilibVy, wpilibOmega);

        // Calculate module states
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Desaturate (normalize) if any wheel exceeds max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.topSpeed_mps);

        // Send to each module with optimization
        for (int i = 0; i < wheels.length; i++) {
            SwerveModuleState optimized = wheels[i].bestAngle(states[i]);
            wheels[i].setAngle(optimized.angle.getDegrees());
            wheels[i].setSpeed(optimized.speedMetersPerSecond);
        }
    }

    public void lockWheels() {
        // X-lock pattern (most common for defense)
        // Adjust angles based on your wheel order: FL, FR, BL, BR
        double[] lockAngles = {45, -45, -45, 45};
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setAngle(lockAngles[i]);
            wheels[i].setSpeed(0);
        }
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Drive.java
    }

<<<<<<< Updated upstream:src/main/java/frc/robot/subsystems/Drivetrain.java
  public SwerveModule[] getWheels() {
    return _wheels;
  }
=======
    @Override
    public void periodic() {
        // Update tipping state here if you have gyro access
        // Example (uncomment when you add gyro back to commands):
        // if (gyro != null) {
        //     double pitch = gyro.getPitch();
        //     isTipping = Math.abs(pitch) > tipThreshold;
        //     tipMagnitude = Math.abs(pitch);
        //     // tipDirection = calculateRecoveryAngle(pitch);
        // }
    }

    public Wheel[] getWheels() {
        return wheels;
    }
>>>>>>> Stashed changes:src/main/java/frc/robot/subsystems/Drive.java
}