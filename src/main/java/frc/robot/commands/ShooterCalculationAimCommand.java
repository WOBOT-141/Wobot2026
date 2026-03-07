// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.ShooterCalculations;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCalculationAimCommand extends Command {

  private double m_targetDistance_in;
  private double m_deltaHeight_in;
  private double m_rimToTargetDistance_in;
  private double m_targetHeight_in;

  private final Shooter m_Shooter;

  /** Creates a new ShooterCalculationAimCommand. */
  public ShooterCalculationAimCommand(double targetDistance_in, double deltaHeight_in, double rimToTargetDistance_in,
      double targetHeight_in, Shooter Shooter) {
    m_targetDistance_in = targetDistance_in;
    m_deltaHeight_in = deltaHeight_in;
    m_rimToTargetDistance_in = rimToTargetDistance_in;
    m_targetHeight_in = targetHeight_in;
    m_Shooter = Shooter;
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
        m_rimToTargetDistance_in, m_targetHeight_in);
    double getLaunchSpeed = ShooterCalculations.getLaunchSpeed(getLaunchAngle, m_rimToTargetDistance_in, m_deltaHeight_in, m_targetHeight_in);
    // shooter.setFlywheelSpeed(getLaunchSpeed)
    // shooter.setHoodAngle(getLaunchAngle)
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
