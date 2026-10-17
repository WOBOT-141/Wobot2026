package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.XYVector;
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.Util.HubConstants;
import frc.robot.subsystems.Vision.VisionServer;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] _wheels;
  private IGyro m_gyro;
  private VisionServer m_visionServer;
  private boolean m_isTipping = false;
  private boolean m_climbing = false;
  private double m_tipMagnitude;
  private double m_tipDirection_deg;
  private double m_tipThreshold = 10;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator;
  private Rotation2d rawGyroRotation = new Rotation2d();

  ChassisSpeeds speeds = new ChassisSpeeds();

  private Pose2d lastPose = new Pose2d();
  private double restAngle_deg;

  @AutoLogOutput
  public int counter = 0;

  SwerveModuleState[] swerveStatesArray;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
          .getStructTopic("MyPoseNTVersion", Pose2d.struct)
          .publish();
  StructArrayPublisher<SwerveModuleState> arrayPublisher = NetworkTableInstance.getDefault()
          .getStructArrayTopic("swerveModuleState", SwerveModuleState.struct).publish();

  private double _lastSimTime = 0;
  private double[] _simDistances;
  private boolean _simInitialized = false;

  RobotConfig config;

  public Drivetrain(SwerveModule[] wheels, IGyro gyro, VisionServer visionServer) {
    _wheels = wheels;
    m_gyro = gyro;
    m_visionServer = visionServer;

        m_kinematics = new SwerveDriveKinematics(getWheelPositions());
        makeOdometryItems(m_kinematics);

        resetWheelPositions();
        swerveStatesArray = new SwerveModuleState[4];
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> getPose(), // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> getCurrentSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> runVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                               // ChassisSpeeds. Also optionally outputs individual
                                                               // module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5,0,0), // Translation PID constants //TODO TUNE
                        new PIDConstants(3,0,0) // Rotation PID constants //TODO TUNE
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    private void makeOdometryItems(SwerveDriveKinematics kinematics) {
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                rawGyroRotation,
                getModulePositions(),
                lastPose,
                VecBuilder.fill(0.05, 0.05, 1.0),
                VecBuilder.fill(0.05, 0.05, 1000.0));

        odometry = new SwerveDriveOdometry(
                kinematics,
                rawGyroRotation,
                getModulePositions());
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

      double resultSpeed_perc = Math.sqrt(Math.pow(resultXY.Y, 2) + Math.pow(resultXY.X, 2)); //This number also changes robot speed

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
        translation *= 1.5; //This number changes scaling (robot speed)

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
    rawGyroRotation = m_gyro.getRotationAs2d();
        poseEstimator.update(rawGyroRotation, getModulePositions());
        Pose2d visionRobotPose;
        Logger.recordOutput("Constants/currentMode", Constants.currentMode.toString());

        switch (Constants.currentMode) {
            case REAL -> {
                Logger.recordOutput("VISIONCLOSESTPOSE", m_visionServer.getTagPose());
                Logger.recordOutput("DistanceToHubCenter", getRobotToHubDist_m());
                visionRobotPose = m_visionServer.getEstimatedRobotPose(m_gyro.getRawGyroAngle());

                if (visionRobotPose != null && m_visionServer.hasTargets()) {
                    Logger.recordOutput("Vision/PoseFromTag", visionRobotPose);

                    // Optimized Loop matching the filtered arrays
                    ArrayList<Pose2d> poses = m_visionServer.getEstimatedRobotPoseFromList();
                    ArrayList<Double> distances = m_visionServer.getAllDetectedTagsDistance();

                    for (int i = 0; i < poses.size(); i++) {
                        addVisionMeasurement(
                                poses.get(i),
                                Timer.getFPGATimestamp(),
                                Units.inchesToMeters(distances.get(i)));
                    }
                }
            }
            case SIM -> {
                visionRobotPose = m_visionServer.getSimulatedRobotPose();

                double currentDistance = getRobotToHubDist_m();
                Logger.recordOutput("Distance from Hub to Robot centers", currentDistance);

                if (visionRobotPose != null) {
                    Logger.recordOutput("Vision/PoseFromTag", visionRobotPose);

                    addVisionMeasurement(
                            visionRobotPose,
                            Timer.getFPGATimestamp(),
                            Units.inchesToMeters(visionRobotPose.getTranslation().getDistance(
                                    m_visionServer.getTagPose(m_visionServer.getClosestSimTagID()).getTranslation())));
                }
            }
            default -> {
                visionRobotPose = m_visionServer.getSimulatedRobotPose();

                double currentDistance = getRobotToHubDist_m();
                Logger.recordOutput("Distance from Hub to Robot centers", currentDistance);

                if (visionRobotPose != null) {
                    Logger.recordOutput("Vision/PoseFromTag", visionRobotPose);
                    ArrayList<Pose2d> poses = m_visionServer.getEstimatedRobotPoseFromList();
                    ArrayList<Double> distances = m_visionServer.getAllDetectedTagsDistance();

                    for (int i = 0; i < poses.size(); i++) {
                        addVisionMeasurement(
                                poses.get(i),
                                Timer.getFPGATimestamp(),
                                Units.inchesToMeters(distances.get(i)));
                    }
                }
            }
        }

        lastPose = poseEstimator.getEstimatedPosition();
        Logger.recordOutput("Odometry/Robot", lastPose);
        Logger.recordOutput("Pose/X", lastPose.getX());
        Logger.recordOutput("Pose/Y", lastPose.getY());
        Logger.recordOutput("Pose/Rotation", lastPose.getRotation().getDegrees());

        Logger.recordOutput("rightface", HubConstants.getRightFace());
        Logger.recordOutput("leftface", HubConstants.getLeftFace());
        Logger.recordOutput("frontface", HubConstants.getNearFace());
        Logger.recordOutput("backface", HubConstants.getFarFace());
  }
  
  public Translation2d[] getWheelPositions() {
        Translation2d[] translations = new Translation2d[_wheels.length];
        for (int i = 0; i < _wheels.length; i++) {
            translations[i] = _wheels[i].getTranslation();
        }
        return translations;
    }

  public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[_wheels.length];
        for (int i = 0; i < _wheels.length; i++) {
            double speedMetersPerSecond = Units.inchesToMeters(_wheels[i].getEncoderDistance()) / 0.02; //check
            Rotation2d angle = Rotation2d.fromDegrees(_wheels[i].getAbsoluteEncoderAngle_deg());
            states[i] = new SwerveModuleState(speedMetersPerSecond, angle);
        }
        return states;
  }

  public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[_wheels.length];

        switch (Constants.currentMode) {
            case REAL -> {
                for (int i = 0; i < _wheels.length; i++) {
                    double distanceMeters = (Units.inchesToMeters(_wheels[i].getEncoderDistance()));
                    Rotation2d azimuth = Rotation2d.fromDegrees(_wheels[i].getAbsoluteEncoderAngle_deg());
                    positions[i] = new SwerveModulePosition(distanceMeters, azimuth);
                }
            }
            case SIM -> {
                for (int i = 0; i < _wheels.length; i++) {
                    positions[i] = new SwerveModulePosition(0.0, new Rotation2d());
                }
            }
            default -> {
                for (int i = 0; i < _wheels.length; i++) {
                    positions[i] = new SwerveModulePosition(0.0, new Rotation2d());
                }
            }
        }

        return positions;
    }

    public Pose2d getPose() {
        return lastPose;
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    public Rotation2d getPoseRotation() {
        return getPose().getRotation();
    }

    public double getHubX() {
        return HubConstants.getTopCenterPoint().getX();
    }

    public double getHubY() {
        return HubConstants.getTopCenterPoint().getY();
    }

    public double getRobotToHubDist_m() {
        double distStep1 = Math.pow(getPoseX() - getHubX(), 2);
        double distStep2 = Math.pow(getPoseY() - getHubY(), 2);
        return Math.sqrt(distStep1 + distStep2);
    }

    public void resetPose(Pose2d startingPose) {
        odometry.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), startingPose);
        poseEstimator.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), startingPose);
        lastPose = startingPose;
    }

    public void resetPose() {
    }

    public void resetWheelPositions() {
        for (SwerveModule wheel : _wheels) {
            wheel.resetEncoder();
        }

        Pose2d resetPose = new Pose2d(0, 0, new Rotation2d()); //STARTING POSITION

        odometry.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), resetPose);
        poseEstimator.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), resetPose);

        lastPose = resetPose;
    }

    public void stop() {
        for (SwerveModule wheel : _wheels) {
            wheel.cancelMovement();
        }
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public double getMaxAngularSpeedRadPerSec() {
        return Constants.DriveConstants.maxSpeedMetersPerSec / Constants.DriveConstants.driveBaseRadius;
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return Constants.DriveConstants.maxSpeedMetersPerSec;
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds); //check later
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DriveConstants.maxSpeedMetersPerSec);
        

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for (int i = 0; i < 4; i++) {
            _wheels[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void driveToPose(Pose2d targetPose) { //NOT USED
        Pose2d error = targetPose.relativeTo(getPose());

        double kP = 0.02; //MAYBE tune

        double vx = error.getX() * kP;
        double vy = error.getY() * kP;
        double omega = error.getRotation().getRadians() * kP;

        runVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, double distanceToTargetMeters) {
        double xyStdDev = 0.5;

        xyStdDev = xyStdDev * Math.pow(distanceToTargetMeters, 2);

        if (xyStdDev < 0.01) {
            xyStdDev = 0.01;
        }

        poseEstimator.addVisionMeasurement(
                visionRobotPose,
                timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, 1.0));
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
