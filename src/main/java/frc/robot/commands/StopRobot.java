// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopRobot extends Command {
  private Intake m_intake;
  private MagazineAndFeed m_magazine;
  private Climber m_climbe;
  private Drivetrain m_drive;
  private Shooter m_shooter;
  /** Creates a new StopRobot. */
  public StopRobot(Intake intake, MagazineAndFeed magazine, Climber climbe, Drivetrain drive, Shooter shooter) {
     m_intake = intake;
     m_magazine = magazine;
     m_climbe = climbe;
     m_drive = drive;
     m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.cancel();
    m_magazine.cancel();
    m_intake.stopArm(); 
    m_intake.intakeRollerStop(); 
    m_climbe.stopClimber(); 
    m_drive.stopMotors();  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
