package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IBotSelector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.lib.SpektrumFlightController;

public class StickCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Drivetrain m_drive;
  private final IGyro m_gyro;
  private final SpektrumFlightController m_flightController;
  private final IBotSelector m_selectedBot;

  private final SlewRateLimiter rampRate = new SlewRateLimiter(1.0);

  private double lastRightDirectional = 0.0;
  private boolean isRotateCenter = false;
  private boolean lockdown = false;

  // Throttle dashboard updates to 5 Hz
  private int dashboardCounter = 0;
  private static final int DASHBOARD_DIVIDER = 10;

  /**
   *
   * @param drive
   *          The subsystem used by this command.
   * @param gyro
   *          TODO
   */
  public StickCommand(
      Drivetrain drive,
      IGyro gyro,
      SpektrumFlightController flightController,
      IBotSelector selectedBot) {

    m_drive = drive;
    m_gyro = gyro;
    m_flightController = flightController;
    m_selectedBot = selectedBot;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    rampRate.reset(0);
    lockdown = false;
  }

  @Override
  public void execute() {

    /* ---------------- Dashboard Debug (throttled) ---------------- */
    if (++dashboardCounter % DASHBOARD_DIVIDER == 0) {
      stickDebug();
    }

    /* ---------------- Read Controller ---------------- */
    double stickX = m_flightController.getDriveX();
    double stickY = m_flightController.getDriveY();
    double stickRotate = m_flightController.getRotate();
    double stickThrottle = m_flightController.getThrottle();

    /* ---------------- Vector Math ---------------- */
    double rightMagnitude = Math.hypot(stickX, stickY);
    double rightDirectional = Math.toDegrees(Math.atan2(stickX, stickY));

    double throttleValue = ((stickThrottle + 1) / 2) + 0.1; // (/2 and .01)

    // Normalize angle
    rightDirectional = (rightDirectional + 360.0) % 360.0;

    /* ---------------- Magnitudes ---------------- */
    double finalLinearMagnitude = rightMagnitude * throttleValue;
    finalLinearMagnitude = Math.max(-1.0, Math.min(1.0, finalLinearMagnitude));

    SmartDashboard.putNumber("finalLinearMagnitude", rightMagnitude * throttleValue);

    double rampedMagnitude = rampRate.calculate(finalLinearMagnitude);

    double finalRotationalMagnitude = stickRotate * throttleValue * 0.85;
    finalRotationalMagnitude = Math.max(-1.0, Math.min(1.0, finalRotationalMagnitude));

    /* ---------------- Lockdown Logic ---------------- */
    if (m_flightController.getSelectPressButton()) {
      lockdown = true;
    }

    if (lockdown) {
      m_drive.lockWheels();

      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {
        lockdown = false;
      }
      return;
    }

    /* ---------------- Drive ---------------- */
    if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {

      double yDist = isRotateCenter ? m_selectedBot.getDistToFront() : 0.0;

      m_drive.driveWithGyro(
          rightDirectional,
          rampedMagnitude,
          finalRotationalMagnitude,
          0.0,
          yDist);

      lastRightDirectional = rightDirectional;

    } else {
      rampRate.reset(0);
      m_drive.driveWithGyro(lastRightDirectional, 0.0, 0.0);
    }
  }

  public void setFrontDrive(boolean isTrue) {
    isRotateCenter = isTrue;
  }

  private void stickDebug() {
    SmartDashboard.putNumber("STICK X", m_flightController.getDriveX());
    SmartDashboard.putNumber("STICK Y", m_flightController.getDriveY());
    SmartDashboard.putNumber("STICK ROT", m_flightController.getRotate());
    SmartDashboard.putNumber("STICK THROTTLE", m_flightController.getThrottle());

  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveWithGyro(lastRightDirectional, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
