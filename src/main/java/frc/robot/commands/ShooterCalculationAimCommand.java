// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Util.ShooterCalculations;
import frc.robot.subsystems.Shooter;

public class ShooterCalculationAimCommand extends Command {

  private final Shooter m_shooter;

  private final double m_targetDistance_in;
  private final double m_deltaHeight_in;
  private final double m_rimToTargetDistance_in;
  private final double m_targetHeightAboveRim_in;

  private double m_launchAngle_deg;
  private double m_launchSpeed_ips;

  public ShooterCalculationAimCommand(
      double targetDistance_in,
      double deltaHeight_in,
      double rimToTargetDistance_in,
      double targetHeightAboveRim_in,
      Shooter shooter) {

    m_targetDistance_in = targetDistance_in;
    m_deltaHeight_in = deltaHeight_in;
    m_rimToTargetDistance_in = rimToTargetDistance_in;
    m_targetHeightAboveRim_in = targetHeightAboveRim_in;

    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {

    m_launchAngle_deg =
        ShooterCalculations.getLaunchAngle(
            m_targetDistance_in,
            m_deltaHeight_in,
            m_rimToTargetDistance_in,
            m_targetHeightAboveRim_in);

    m_launchSpeed_ips =
        ShooterCalculations.getLaunchSpeed(
            m_launchAngle_deg,
            m_targetDistance_in,
            m_rimToTargetDistance_in,
            m_targetHeightAboveRim_in);

    m_shooter.setShooterVelocity(m_launchSpeed_ips);
    m_shooter.setHoodAngle(m_launchAngle_deg);
  }

  @Override
  public void execute() {
    // Nothing needed — shooter continues running
  }

  @Override
  public void end(boolean interrupted) {

    if (interrupted) {
      m_shooter.cancel();
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
