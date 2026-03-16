// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class AxisIntakeRunCommand extends Command {
  
  private final Intake m_intake;
  private final DoubleSupplier m_leftTriggerAxisSupplier;

  public AxisIntakeRunCommand(Intake intake, DoubleSupplier leftTriggerAxisSupplier) {
     m_intake = intake;
    m_leftTriggerAxisSupplier = leftTriggerAxisSupplier;
   
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
    double axisRead = m_leftTriggerAxisSupplier.getAsDouble();

    // Clamp the axis to a safe range
    axisRead = Math.max(0.0, Math.min(1.0, axisRead));

    SmartDashboard.putNumber("Intake Axis", axisRead);
    
    if (axisRead < IntakeConstants.intakeDeadband_perc) {
      m_intake.intakeRollerStop();
    } else if (axisRead < IntakeConstants.intakeFastThreshold_perc) {
      m_intake.intakeRun(IntakeConstants.intakeSlowSpeed_perc);
    } else {
      m_intake.intakeRun(IntakeConstants.intakeFastSpeed_perc);
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
