// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRunCommand extends Command {
  /** Creates a new IntakeRunCommand. */
  private Intake m_intake;
  private final RobotContainer m_robotContainer;
  

  public IntakeRunCommand(Intake intake, RobotContainer robotContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotContainer = robotContainer;
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

  if ( m_robotContainer.getLeftTriggerAxis() <= .89) {
      m_intake.intakeRun(-.8);
    }
  else if(m_robotContainer.getLeftTriggerAxis() > 9){
    m_intake.intakeRun(-1);
  }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intakeRollerStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
