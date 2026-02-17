package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot.java should be minimal.
 *
 * RobotContainer owns:
 * - Subsystems
 * - Default commands
 * - Button bindings
 * - Autonomous selection
 *
 * Robot.java only manages mode transitions.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up.
   */
  public Robot() {
    // RobotContainer sets up everything automatically.
    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs every 20ms in all modes.
   * Required for CommandScheduler to function.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
    // Re-zero swerve modules while disabled
    m_robotContainer.wheelSetup();
  }

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {

    // Cancel autonomous when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
