// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunUntilCurrentLimit extends Command {

  Intake m_intake;
  double m_currentLimit;
  boolean m_leftAtLimit;
  boolean m_rightAtLimit;
  double m_speed;
  /** Creates a new RunUntilCurrentLimit. */
  public RunUntilCurrentLimit(double speed, double currentLimit, Intake intake) {
    m_intake = intake;
    m_speed = speed;
    m_currentLimit = currentLimit;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RunUntilCurrentLimit START");
    m_leftAtLimit = false;
    m_rightAtLimit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_leftAtLimit)
      m_leftAtLimit = m_intake.runLeftUntilCurrentLimit(m_speed, m_currentLimit);

    if (!m_rightAtLimit)
      m_rightAtLimit = m_intake.runRightUntilCurrentLimit(m_speed, m_currentLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_leftAtLimit && m_rightAtLimit;
  }
}
