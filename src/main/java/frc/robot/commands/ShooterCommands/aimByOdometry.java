// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Util.ShooterCalculations;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class aimByOdometry extends Command {

  // Subsystem Dependencies
  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;

  // Shooter Calculation Constants
  private final double m_deltaHeight_in;
  private final double m_rimToTargetDistance_in;
  private final double m_targetHeightAboveRim_in;

  // PID Controller for rotating the chassis to face the hub
  // P, I, and D values should be tuned based on your specific robot's turning
  // dynamics
  private final PIDController m_turnController = new PIDController(Constants.RotateCommandConstants.P,
      Constants.RotateCommandConstants.I, Constants.RotateCommandConstants.D);

  /**
   * Creates a new aimByOdometry command.
   * This command continuously updates the shooter's hood and flywheel based on
   * the
   * robot's distance to the hub, and simultaneously rotates the drivetrain to
   * face the hub.
   * * @param drivetrain The drivetrain subsystem used for odometry and chassis
   * rotation.
   * * @param shooter                 The shooter subsystem to spool and angle.
   * @param deltaHeight_in          Vertical distance between the shooter exit and
   * the target.
   * @param rimToTargetDistance_in  Horizontal distance from the front rim of the
   * hub to the actual target center.
   * @param targetHeightAboveRim_in Vertical distance from the rim of the hub to
   * the target plane.
   */
  public aimByOdometry(
      Drivetrain drivetrain,
      Shooter shooter,
      double deltaHeight_in,
      double rimToTargetDistance_in,
      double targetHeightAboveRim_in) {

    m_drivetrain = drivetrain;
    m_shooter = shooter;

    m_deltaHeight_in = deltaHeight_in;
    m_rimToTargetDistance_in = rimToTargetDistance_in;
    m_targetHeightAboveRim_in = targetHeightAboveRim_in;

    // Require both subsystems since we are controlling the shooter and the
    // drivetrain's rotation
    addRequirements(m_drivetrain, m_shooter);

    // Enable continuous input for the turning PID so the robot takes the shortest
    // path
    // when crossing the 180 / -180 degree boundary.
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);

    // Set a tolerance of 1.5 degrees for the aiming rotation
    m_turnController.setTolerance(Math.toRadians(1.5)); //Can prob increase to 3
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the PID controller to clear any integral windup from previous runs
    m_turnController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceToHub_m = m_drivetrain.getRobotToHubDist_m();

    double currentDistance_in = Units.metersToInches(distanceToHub_m);

    double targetLaunchAngle = ShooterCalculations.getLaunchAngle(
        currentDistance_in,
        m_deltaHeight_in,
        m_rimToTargetDistance_in,
        m_targetHeightAboveRim_in);

    double targetLaunchSpeed = ShooterCalculations.getLaunchSpeed(
        targetLaunchAngle,
        currentDistance_in,
        m_rimToTargetDistance_in,
        m_targetHeightAboveRim_in);

    // Apply the calculated values to the Shooter subsystem
    m_shooter.setShooterVelocity(targetLaunchSpeed);
    m_shooter.setHoodAngle(targetLaunchAngle);

    // Find the robot's current position on the field
    double robotX = m_drivetrain.getPoseX();
    double robotY = m_drivetrain.getPoseY();

    // Find the hub's position on the field
    double hubX = m_drivetrain.getHubX();
    double hubY = m_drivetrain.getHubY();

    // Calculate the angle required to face the hub.
    // Math.atan2 takes (y, x) and returns radians from -PI to PI.
    double targetAngleRadians = Math.atan2(hubY - robotY, hubX - robotX);

    // Get the robot's current heading in radians
    double currentAngleRadians = m_drivetrain.getPoseRotation().getRadians();

    // Calculate the required rotational velocity using the PID controller
    double rotationSpeedRadPerSec = m_turnController.calculate(currentAngleRadians, targetAngleRadians);

    // If we are already pointing at the target within our tolerance, stop rotating
    // to prevent jitter
    if (m_turnController.atSetpoint()) {
      rotationSpeedRadPerSec = 0.0;
    }

    // Command the drivetrain to rotate while keeping translation at 0.
    // If you want the driver to still be able to move while aiming, you would
    // replace
    // the 0.0s here with inputs from your joystick suppliers.
    ChassisSpeeds aimSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        0.0, // Translation X speed (Meters/Sec)
        0.0, // Translation Y speed (Meters/Sec)
        rotationSpeedRadPerSec, // Rotational speed (Radians/Sec)
        m_drivetrain.getPoseRotation() // Current robot heading
    );

    // Feed the speeds to the drivetrain
    m_drivetrain.runVelocity(aimSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the shooter and the drivetrain when the command finishes or the driver
    // releases the button
    m_shooter.cancel();
    m_drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command typically runs as a "while held" command bound to a button.
    // Therefore, it should not finish on its own unless explicitly designed to fire
    // and stop.
    return false;
  }
}