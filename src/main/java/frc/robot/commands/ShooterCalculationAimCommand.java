// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Util.ShooterCalculations;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCalculationAimCommand extends Command {

  private double m_targetDistance_in;
  private double m_deltaHeight_in;
  private double m_rimToTargetDistance_in;
  private double m_targetHeightAboveRim_in;

  private final Shooter m_shooter;
  private final CommandXboxController m_controller;

  /** Creates a new ShooterCalculationAimCommand. */
  public ShooterCalculationAimCommand(double targetDistance_in, double deltaHeight_in, double rimToTargetDistance_in,
      double targetHeightAboveRim_in, CommandXboxController controller, Shooter shooter) {
    m_targetDistance_in = targetDistance_in;
    m_deltaHeight_in = deltaHeight_in;
    m_rimToTargetDistance_in = rimToTargetDistance_in;
    m_targetHeightAboveRim_in = targetHeightAboveRim_in;
    m_controller = controller;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double getLaunchAngle = ShooterCalculations.getLaunchAngle(m_targetDistance_in, m_deltaHeight_in,
        m_rimToTargetDistance_in, m_targetHeightAboveRim_in);
    double getLaunchSpeed = ShooterCalculations.getLaunchSpeed(getLaunchAngle, m_targetDistance_in, m_rimToTargetDistance_in, m_targetHeightAboveRim_in);
    m_shooter.setShooterAngleAndVelocity(getLaunchAngle, getLaunchSpeed);
    
    //Controller Feedback
    double rumble = m_shooter.isReadyToShoot() ? ControllerConstants.rumbleFeedback_perc : 0.0;
    m_controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kBothRumble, 0);
    m_shooter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
