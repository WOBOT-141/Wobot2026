package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.XYVector;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Vision.VisionServer;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule[] _wheels;
    private final IGyro m_gyro;
    private final VisionServer m_visionServer;

    private boolean m_isTipping = false;
    private boolean m_climbing = false;
    private double pitchAngle;
    private double rollAngle;
    private double m_tipMagnitude;
    private double m_tipDirection_deg;
    private double m_tipThreshold = 10;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d rawGyroRotation = new Rotation2d();

    private Pose2d lastPose = new Pose2d();
    private double restAngle_deg;

    @AutoLogOutput
    public int counter = 0;

    SwerveModuleState[] swerveStatesArray;

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPoseNTVersion", Pose2d.struct)
            .publish();
    StructArrayPublisher<SwerveModuleState> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("swerveModuleState", SwerveModuleState.struct).publish();

    private double _lastSimTime = 0;
    private double[] _simDistances;
    private boolean _simInitialized = false;

    public Drivetrain(SwerveModule[] wheels, IGyro gyro, VisionServer visionServer) {
        _wheels = wheels;
        m_gyro = gyro;
        m_visionServer = visionServer;

        m_kinematics = new SwerveDriveKinematics(getWheelPositions());
        makeOdometryItems(m_kinematics);

        resetWheelPositions();
        swerveStatesArray = new SwerveModuleState[4];
    }

    private void makeOdometryItems(SwerveDriveKinematics kinematics) {
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                rawGyroRotation,
                getModulePositions(),
                lastPose);

        odometry = new SwerveDriveOdometry(
                kinematics,
                rawGyroRotation,
                getModulePositions());
    }

    public void antiTip() {
        pitchAngle = m_gyro.getPitch();
        rollAngle = m_gyro.getRoll();

        m_tipMagnitude = Math.sqrt(pitchAngle * pitchAngle + rollAngle * rollAngle);
        m_tipDirection_deg = Math.toDegrees(Math.atan2(rollAngle, pitchAngle)) + 90;

        m_isTipping = m_tipMagnitude > m_tipThreshold;
    }

    public void setClimbing(boolean climb) {
        m_climbing = climb;
    }

    public void driveWithoutGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc) {
        driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, 0, 0);
    }

    public void driveWithoutGyro(double translationAngle_deg,
            double translationSpeed_perc,
            double rotationalSpeed_perc,
            double rotX, double rotY) {

        if (m_isTipping && !m_climbing) {
            translationAngle_deg = m_tipDirection_deg;
            translationSpeed_perc = m_tipMagnitude / (m_tipThreshold * 2);
            rotationalSpeed_perc = 0;
            rotX = 0;
            rotY = 0;
        }

        double[] calcWheelAngle = new double[_wheels.length];
        double[] calcWheelSpeed = new double[_wheels.length];
        double normalizingFactor = 1;

        if (translationAngle_deg == 0 && translationSpeed_perc == 0 && rotationalSpeed_perc == 0) {
            stop();
            return;
        }

        for (int i = 0; i < _wheels.length; ++i) {
            double wheelX = _wheels[i].get_xPos();
            double wheelY = _wheels[i].get_yPos();

            double deltaX = wheelX - rotX;
            double deltaY = wheelY - rotY;

            XYVector linearVector = getTranslationVector(translationSpeed_perc, translationAngle_deg);
            double rotationAngle = getRotationAngle(deltaX, deltaY, translationAngle_deg, rotationalSpeed_perc);
            XYVector rotationVector = getRotationVector(rotationAngle, rotationalSpeed_perc, deltaX, deltaY);

            XYVector resultXY = new XYVector(
                    linearVector.X + rotationVector.X,
                    linearVector.Y + rotationVector.Y);

            double resultSpeed = Math.hypot(resultXY.X, resultXY.Y);
            double resultAngle = getResultAngle(resultXY);

            calcWheelAngle[i] = resultAngle;
            calcWheelSpeed[i] = resultSpeed;
            normalizingFactor = Math.max(normalizingFactor, resultSpeed);
        }

        for (int i = 0; i < _wheels.length; i++) {

            var inputState = new SwerveModuleState(
                    (calcWheelSpeed[i] / normalizingFactor),
                    Rotation2d.fromDegrees(calcWheelAngle[i]));

            var outputState = _wheels[i].bestAngle(inputState);

            _wheels[i].setAngle(outputState.angle.getDegrees());
            _wheels[i].setSpeed(outputState.speedMetersPerSecond);

            Logger.recordOutput("wheel " + i + " angle", outputState.angle.getDegrees());
            Logger.recordOutput("wheel " + i + " speed", outputState.speedMetersPerSecond);
        }
    }

    public void driveWithGyro(double translationAngle_deg,
            double translationSpeed_perc,
            double rotationalSpeed_perc,
            double rotX, double rotY) {
        translationAngle_deg -= m_gyro.getRealGyroAngle();
        driveWithoutGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, rotX, rotY);
    }

    public void driveWithGyro(double translationAngle_deg, double translationSpeed_perc, double rotationalSpeed_perc) {
        driveWithGyro(translationAngle_deg, translationSpeed_perc, rotationalSpeed_perc, 0, 0);
    }

    public double getResultAngle(XYVector resultXY) {
        double resultAngle_deg = Math.toDegrees(Math.atan2(resultXY.Y, resultXY.X));
        resultAngle_deg = -(90 - resultAngle_deg);

        if (resultXY.X == 0 && resultXY.Y == 0) {
            resultAngle_deg = restAngle_deg;
        } else {
            restAngle_deg = resultAngle_deg;
        }
        return resultAngle_deg;
    }

    public XYVector getRotationVector(double rotationAngle_deg, double rotationalSpeed_perc,
            double deltaX, double deltaY) {
        double y = Math.cos(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_perc);
        double x = Math.sin(Math.toRadians(rotationAngle_deg)) * Math.abs(rotationalSpeed_perc);

        if (deltaX == 0 && deltaY == 0) {
            x = 0;
            y = 0;
        }
        return new XYVector(x, y);
    }

    public XYVector getTranslationVector(double translationSpeed_perc, double translationAngle_deg) {
        double y = Math.cos(Math.toRadians(translationAngle_deg)) * translationSpeed_perc;
        double x = Math.sin(Math.toRadians(translationAngle_deg)) * translationSpeed_perc;
        return new XYVector(x, y);
    }

    public double getRotationAngle(double deltaX, double deltaY,
            double translationAngle_deg,
            double rotationalSpeed_perc) {

        double alpha;
        if (deltaY == 0 && deltaX > 0) {
            alpha = 90;
        } else if (deltaY == 0 && deltaX < 0) {
            alpha = -90;
        } else if (deltaY == 0 && deltaX == 0) {
            alpha = translationAngle_deg;
        } else {
            alpha = Math.toDegrees(Math.atan2(deltaX, deltaY));
        }

        if (rotationalSpeed_perc > 0) {
            return alpha + 90;
        }
        if (rotationalSpeed_perc < 0) {
            return alpha - 90;
        }
        return alpha;
    }

    @Override
    public void periodic() {
        rawGyroRotation = m_gyro.getRotationAs2d();
        poseEstimator.update(rawGyroRotation, getModulePositions());//Update the odometry without vision first

        if (Constants.currentMode == Mode.SIM) {//this is something the code i was looking at had 
            Pose2d visionRobotPose = m_visionServer.getSimulatedRobotPose();//get the estimatedpose from vision

            if (visionRobotPose != null) {
                Logger.recordOutput("Vision/PoseFromTag", visionRobotPose);//output our estimated position so we can see the deviation from our actual to the vision estimate

                poseEstimator.addVisionMeasurement(visionRobotPose, Timer.getFPGATimestamp());//this is so we dont try to update based on old data 
            }
        } else if (Constants.currentMode == Mode.REAL) {
            Pose2d visionRobotPose = m_visionServer.getEstimatedRobotPose();//get the estimatedpose from vision

            if (visionRobotPose != null) {
                Logger.recordOutput("Vision/PoseFromTag", visionRobotPose);//output our estimated position so we can see the deviation from our actual to the vision estimate

                poseEstimator.addVisionMeasurement(visionRobotPose, Timer.getFPGATimestamp());//this is so we dont try to update based on old data 
            }
        }

        lastPose = poseEstimator.getEstimatedPosition();// update our pose 
        Logger.recordOutput("Odometry/Robot", lastPose);//update our pose on AdvantageScope
        Logger.recordOutput("Pose/X", lastPose.getX());//List  our X,Y, and current rotation from our pose 
        Logger.recordOutput("Pose/Y", lastPose.getY());
        Logger.recordOutput("Pose/Rotation", lastPose.getRotation().getDegrees());

        Logger.recordOutput("rightface", Constants.Hub.getRightFace());
        Logger.recordOutput("leftface", Constants.Hub.getLeftFace());
        Logger.recordOutput("frontface", Constants.Hub.getNearFace());
        Logger.recordOutput("backface", Constants.Hub.getFarFace());


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
            double speedMetersPerSecond = Units.inchesToMeters(_wheels[i].getEncoderDistance()) / 0.02;
            Rotation2d angle = Rotation2d.fromDegrees(_wheels[i].getAbsoluteEncoderAngle_deg());
            states[i] = new SwerveModuleState(speedMetersPerSecond, angle);
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[_wheels.length];

        if (Constants.currentMode == Constants.Mode.REAL) {
            for (int i = 0; i < _wheels.length; i++) {
                double distanceMeters = Units.inchesToMeters(_wheels[i].getEncoderDistance());
                Rotation2d azimuth = Rotation2d.fromDegrees(_wheels[i].getAbsoluteEncoderAngle_deg());
                positions[i] = new SwerveModulePosition(distanceMeters, azimuth);
            }
        } else {
            // For simulation, provide default positions
            for (int i = 0; i < _wheels.length; i++) {
                positions[i] = new SwerveModulePosition(0.0, new Rotation2d());
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


    public void resetWheelPositions() {
        for (SwerveModule wheel : _wheels) {
            wheel.resetEncoder();
        }

        Pose2d resetPose = new Pose2d(0, 0, new Rotation2d());

        odometry.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), resetPose);
        poseEstimator.resetPosition(m_gyro.getRotationAs2d(), getModulePositions(), resetPose);

        lastPose = resetPose;
    }

    public void stop() {
        for (SwerveModule wheel : _wheels) {
            wheel.cancelMovement();
        }
    }

    public void stopMotors() {
        driveWithoutGyro(0, 0, 0);
    }

    public void lockWheels() {
        for (int i = 0; i < _wheels.length; i++) {
            _wheels[i].setAngle((i * 90) + 45);
            _wheels[i].setSpeed(0);
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

    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DriveConstants.maxSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            _wheels[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void driveToPose(Pose2d targetPose) {
        Pose2d error = targetPose.relativeTo(getPose());

        double kP = 2.0;

        double vx = error.getX() * kP;
        double vy = error.getY() * kP;
        double omega = error.getRotation().getRadians() * kP;

        runVelocity(new ChassisSpeeds(vx, vy, omega));
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
