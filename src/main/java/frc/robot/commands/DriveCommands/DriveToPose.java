package frc.robot.commands.DriveCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.LoggedTunableNumber;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber timeout =
      new LoggedTunableNumber("DriveToPose/timeout", 2.0);

  private static final LoggedTunableNumber kPX = new LoggedTunableNumber("DriveToPose/kPX", 2.5);
  private static final LoggedTunableNumber kPY = new LoggedTunableNumber("DriveToPose/kPY", 2.5);
  private static final LoggedTunableNumber kPTheta =
      new LoggedTunableNumber("DriveToPose/kPTheta", 4.0);

  private static final LoggedTunableNumber maxVelXY =
      new LoggedTunableNumber("DriveToPose/MaxVelXY", 2.0);
  private static final LoggedTunableNumber maxAccelXY =
      new LoggedTunableNumber("DriveToPose/MaxAccelXY", 2.0);
  private static final LoggedTunableNumber maxVelTheta =
      new LoggedTunableNumber("DriveToPose/MaxVelThetaDeg", 180.0);
  private static final LoggedTunableNumber maxAccelTheta =
      new LoggedTunableNumber("DriveToPose/MaxAccelThetaDeg", 360.0);

  private final double speedScalar;
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;

  private Pose2d targetPose;
  private Pose2d currentPose;
  private Timer timeoutTimer = new Timer();

  private ProfiledPIDController driveControllerX;
  private ProfiledPIDController driveControllerY;
  private ProfiledPIDController thetaController;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier, double speedScalar) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.speedScalar = MathUtil.clamp(speedScalar, 0.0, 1.0);
    addRequirements(drivetrain);
  }

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this(drivetrain, poseSupplier, 1.0);
  }

  @Override
  public void initialize() {
    createControllers();
    currentPose = drivetrain.getPose();

    Logger.recordOutput("DriveToPose/Init/StartPose", currentPose);
    Logger.recordOutput("DriveToPose/Init/SpeedScalar", speedScalar);

    driveControllerX.reset(currentPose.getX(), 0);
    driveControllerY.reset(currentPose.getY(), 0);
    thetaController.reset(currentPose.getRotation().getRadians(), 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose = poseSupplier.get();
    driveControllerX.setGoal(targetPose.getX());
    driveControllerY.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    Logger.recordOutput("DriveToPose/Init/TargetPose", targetPose);

    driveControllerX.setTolerance(Units.inchesToMeters(0.5));
    driveControllerY.setTolerance(Units.inchesToMeters(0.5));
    thetaController.setTolerance(Units.degreesToRadians(0.1));

    timeoutTimer.restart();
  }

  @Override
  public void execute() {
    currentPose = drivetrain.getPose();

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double thetaError =
        targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    Logger.recordOutput("DriveToPose/Error/XErrorMeters", xError);
    Logger.recordOutput("DriveToPose/Error/YErrorMeters", yError);
    Logger.recordOutput("DriveToPose/Error/ThetaErrorDegrees", Units.radiansToDegrees(thetaError));

    double driveXVelocity = driveControllerX.calculate(currentPose.getX());
    double driveYVelocity = driveControllerY.calculate(currentPose.getY());
    double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians());

    double scaledXVelocity = driveXVelocity * speedScalar;
    double scaledYVelocity = driveYVelocity * speedScalar;
    double scaledThetaVelocity = thetaVelocity * speedScalar;

    if (thetaController.atGoal()) {
      scaledThetaVelocity = 0.0;
    }

    Logger.recordOutput("DriveToPose/VelocityCommands/X", scaledXVelocity);
    Logger.recordOutput("DriveToPose/VelocityCommands/Y", scaledYVelocity);
    Logger.recordOutput(
        "DriveToPose/VelocityCommands/Theta", Units.radiansToDegrees(scaledThetaVelocity));

    drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            scaledXVelocity, scaledYVelocity, scaledThetaVelocity, drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timeoutTimer.stop();

    Logger.recordOutput("DriveToPose/End/FinalPose", drivetrain.getPose());
    Logger.recordOutput("DriveToPose/End/TotalTime", timeoutTimer.get());
    Logger.recordOutput("DriveToPose/End/WasInterrupted", interrupted);
    Logger.recordOutput("DriveToPose/End/TimedOut", timeoutTimer.hasElapsed(timeout.get()));
  }

  @Override
  public boolean isFinished() {
    boolean finished = atGoal() || timeoutTimer.hasElapsed(timeout.get());

    Logger.recordOutput("DriveToPose/IsFinished", finished);
    Logger.recordOutput("DriveToPose/CompletionReason", atGoal() ? "Reached Target" : "Timed Out");

    return finished;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveControllerX.atGoal() && driveControllerY.atGoal() && thetaController.atGoal();
  }

  private void createControllers() {
    driveControllerX =
        new ProfiledPIDController(
            kPX.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(maxVelXY.get(), maxAccelXY.get()));

    driveControllerY =
        new ProfiledPIDController(
            kPY.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(maxVelXY.get(), maxAccelXY.get()));

    thetaController =
        new ProfiledPIDController(
            kPTheta.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Math.toRadians(maxVelTheta.get()), Math.toRadians(maxAccelTheta.get())));
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }
}