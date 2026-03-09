package frc.robot.commands.DriveCommands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IBotSelector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

/**
 * StickCommand
 *
 */
public class StickCommand extends Command {

  private final Drivetrain m_drive;
  private final IBotSelector m_selectedBot;

  // Axis suppliers (injected from RobotContainer)
  private final DoubleSupplier m_driveXSupplier;
  private final DoubleSupplier m_driveYSupplier;
  private final DoubleSupplier m_rotateSupplier;
  private final DoubleSupplier m_throttleSupplier;
  private final BooleanSupplier m_selectButtonSupplier;

  // Ramp only linear magnitude (as original code did)
  private final SlewRateLimiter m_rampRate = new SlewRateLimiter(1.0);

  private double m_lastRightDirectional = 0.0;
  private boolean m_isRotateCenter = false;
  private boolean m_lockdown = false;
  private Optional<Double> m_altCommandedRotation_perc = Optional.empty();

  // Throttle dashboard updates to 5 Hz
  private int m_dashboardCounter = 0;
  private static final int DASHBOARD_DIVIDER = 10;

  public StickCommand(
      Drivetrain drive,
      IGyro gyro,
      IBotSelector selectedBot,
      DoubleSupplier driveXSupplier,
      DoubleSupplier driveYSupplier,
      DoubleSupplier rotateSupplier,
      DoubleSupplier throttleSupplier,
      BooleanSupplier selectButtonSupplier) {

    m_drive = drive;
    m_selectedBot = selectedBot;
    m_driveXSupplier = driveXSupplier;
    m_driveYSupplier = driveYSupplier;
    m_rotateSupplier = rotateSupplier;
    m_throttleSupplier = throttleSupplier;
    m_selectButtonSupplier = selectButtonSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_rampRate.reset(0);
    m_lockdown = false;
  }

  @Override
  public void execute() {

    /* ---------------- Dashboard Debug (throttled) ---------------- */
    if (++m_dashboardCounter % DASHBOARD_DIVIDER == 0) {
      stickDebug();
    }

    /* ---------------- Read Controller ---------------- */
    double stickX = m_driveXSupplier.getAsDouble();
    double stickY = m_driveYSupplier.getAsDouble();
    double stickRotate = m_rotateSupplier.getAsDouble();
    double stickThrottle = m_throttleSupplier.getAsDouble();

    /* ---------------- Vector Math ---------------- */

    // Convert X/Y to magnitude and direction (polar form)
    double rightMagnitude = Math.hypot(stickX, stickY);

    double rightDirectional = Math.toDegrees(Math.atan2(stickX, stickY));

    // Convert throttle from [-1,1] to [0,1] with slight boost
    double throttleValue = ((stickThrottle + 1) / 2.0) + 0.1;

    // Slow the bot down if we are aiming at something
    if (m_altCommandedRotation_perc.isPresent()){
      throttleValue = throttleValue / 2.0;
    }

    // Normalize angle to 0-360
    rightDirectional = (rightDirectional + 360.0) % 360.0;

    /* ---------------- Magnitudes ---------------- */

    double finalLinearMagnitude = rightMagnitude * throttleValue;
    finalLinearMagnitude = Math.max(-1.0, Math.min(1.0, finalLinearMagnitude));

    double rampedMagnitude = m_rampRate.calculate(finalLinearMagnitude);

    double finalRotationalMagnitude = stickRotate * throttleValue * 0.85;
    finalRotationalMagnitude =
        Math.max(-1.0, Math.min(1.0, finalRotationalMagnitude));

    /* ---------------- Lockdown Logic ---------------- */

    if (m_selectButtonSupplier.getAsBoolean()) {
      m_lockdown = true;
    }

    if (m_lockdown) {
      m_drive.lockWheels();

      // Exit lockdown if driver commands movement
      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {
        m_lockdown = false;
      }
      return;
    }

    /* ---------------- Override Rotation --------------- */
    if (finalRotationalMagnitude == 0 && m_altCommandedRotation_perc.isPresent()) {
      finalRotationalMagnitude = m_altCommandedRotation_perc.get();
    }

    /* ---------------- Drive ---------------- */

    if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {

      double yDist = m_isRotateCenter
          ? m_selectedBot.getDistToFront()
          : 0.0;

      m_drive.driveWithGyro(
          rightDirectional,
          rampedMagnitude,
          finalRotationalMagnitude,
          0.0,
          yDist);

      m_lastRightDirectional = rightDirectional;

    } else {
      // Maintain last heading when stopped
      m_rampRate.reset(0);
      m_drive.driveWithGyro(m_lastRightDirectional, 0.0, 0.0);
    }
  }

  public void setFrontDrive(boolean isTrue) {
    m_isRotateCenter = isTrue;
  }

  public void setAlternateRotation(Optional<Double> altRotation_perc){
    m_altCommandedRotation_perc = altRotation_perc;
  }

  private void stickDebug() {
    SmartDashboard.putNumber("STICK X", m_driveXSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK Y", m_driveYSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK ROT", m_rotateSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK THROTTLE", m_throttleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveWithGyro(m_lastRightDirectional, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
