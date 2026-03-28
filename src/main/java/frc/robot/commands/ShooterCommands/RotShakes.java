// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagazineAndFeed;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotShakes extends Command {
  private final Drivetrain m_drivetrain;
  private final MagazineAndFeed m_magazine;
  private double startTime;
  private final double m_speed;

  /** Creates a new ShakingBot. */
  public RotShakes(double speed, Drivetrain drivetrain, MagazineAndFeed magazine) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_magazine = magazine;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp() - startTime;
    if (currentTime < 0.5) {
      m_drivetrain.driveWithoutGyro(0, -0.3, 0);
      m_magazine.setFeedVelocity(-0.3);
      return;
    }
    if ((currentTime % 0.7) < 0.35) {
      m_drivetrain.driveWithoutGyro(0, 0, m_speed);
      m_magazine.setFeedVelocity(-0.3);
    } else {
      m_drivetrain.driveWithoutGyro(0, 0, -1 * m_speed);
      m_magazine.setFeedVelocity(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveWithoutGyro(0, 0, 0);
    m_magazine.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
