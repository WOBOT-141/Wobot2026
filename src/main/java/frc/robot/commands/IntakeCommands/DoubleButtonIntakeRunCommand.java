// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DoubleButtonIntakeRunCommand extends Command {
  /** Creates a new IntakeRunCommand. */
  private final Intake m_intake;
  private final DoubleSupplier m_maxAxisSupplier;

  public DoubleButtonIntakeRunCommand(Intake intake, DoubleSupplier maxAxisSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_maxAxisSupplier = maxAxisSupplier;
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double maxAxisRead = m_maxAxisSupplier.getAsDouble();
    maxAxisRead = Math.max(0.0, Math.min(1.0, maxAxisRead));

    double speed = maxAxisRead >= 0.5 ? IntakeConstants.intakeFastSpeed_perc : IntakeConstants.intakeSlowSpeed_perc;
    m_intake.intakeRun(speed);
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
