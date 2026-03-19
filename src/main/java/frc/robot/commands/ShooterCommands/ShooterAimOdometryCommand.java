// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Util.ShooterCalculations;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.VisionServer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterAimOdometryCommand extends Command {

    /**
     * Creates a new ShooterAimOdometryCommand.
     */

    VisionServer m_visionServer;
    Drivetrain m_drive;
    Shooter m_shooter;

    double targetAngle;
    double targetDistance;
    double launchSpeed;

    public ShooterAimOdometryCommand(VisionServer visionServer, Drivetrain drive, Shooter shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_visionServer = visionServer;
        m_drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Pose2d robotPose = m_drive.getPose();

        Translation2d targetLocation = Constants.Hub.getTopCenterPoint().toTranslation2d();

        double distanceMeters = robotPose.getTranslation().getDistance(targetLocation);

        double distanceInches = Units.metersToInches(distanceMeters);

        targetDistance = distanceInches;

        targetAngle = ShooterCalculations.getLaunchAngle(targetDistance);
        launchSpeed = ShooterCalculations.getLaunchSpeed(targetAngle, targetDistance, 10, 24);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_shooter.setHoodAngle(targetAngle);
      m_shooter.setShooterVelocity(launchSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
