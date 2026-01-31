// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MultiStickCommand extends Command {
  private static Boolean _initialized = false;
  private static SendableChooser<Command> _commandChooser;

  /** Creates a new MultiStickCommand. */
  public MultiStickCommand(NormalizedStickCommand normalizedCommand, StickCommand regularCommand, ConsolidatedStickCommand consolidatedStickCommand) {
    addRequirements(regularCommand.getRequirements());

    if (!_initialized) {
      _commandChooser = new SendableChooser<Command>();
      _commandChooser.setDefaultOption("Regular Stick", regularCommand);
      _commandChooser.addOption("Normalized Stick", normalizedCommand);
      _commandChooser.addOption("Consolidated Stick (new)", consolidatedStickCommand);

      SmartDashboard.putData(_commandChooser);
      System.out.println("Command chooser initialized");

      _initialized = true;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command command = _commandChooser.getSelected();
    command.execute();
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
