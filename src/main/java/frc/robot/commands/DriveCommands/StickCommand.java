package frc.robot.commands.DriveCommands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IBotSelector;
import frc.robot.Constants.WheelConstants;
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

  // Ramp only linear magnitude (as original code did)
  private final SlewRateLimiter m_rampRate = new SlewRateLimiter(1.0);

  private double m_lastTranslationAngle_deg = 0.0;
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
      DoubleSupplier throttleSupplier) {

    m_drive = drive;
    m_selectedBot = selectedBot;
    m_driveXSupplier = driveXSupplier;
    m_driveYSupplier = driveYSupplier;
    m_rotateSupplier = rotateSupplier;
    m_throttleSupplier = throttleSupplier;

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
    double stickX_perc = m_driveXSupplier.getAsDouble();
    double stickY_perc = m_driveYSupplier.getAsDouble();
    double stickRotate_perc = m_rotateSupplier.getAsDouble();
    double stickThrottle_perc = m_throttleSupplier.getAsDouble();

    if (stickX_perc == 0 && stickY_perc == 0 && stickRotate_perc == 0){
      if (m_lockdown) {
        m_drive.lockWheels();
        return;
      }
        
      if (m_altCommandedRotation_perc.isEmpty() || m_altCommandedRotation_perc.get().isNaN()) {
        // Maintain last heading when the sticks are at 0, the wheels aren't being locked, and there is no commanded rotation.
        m_rampRate.reset(0);
        m_drive.driveWithGyro(m_lastTranslationAngle_deg, 0.0, 0.0);
        return;
      }
    } else {
      m_lockdown = false; // Exit lockdown if driver commands movement
    }

    /* ---------------- Vector Math ---------------- */
    double throttle_perc = calcThrottle_perc(stickThrottle_perc);
    double translationMagnitude_perc = throttle_perc * calcTranslationMagnitude_perc(stickX_perc, stickY_perc);
    double translationAngle_deg = calcTranslationAngle_deg(stickX_perc, stickY_perc);
    double rotationalMagnitude_perc = throttle_perc * stickRotate_perc;

    /* ---------------- Override Rotation --------------- */
    if (rotationalMagnitude_perc == 0 && m_altCommandedRotation_perc.isPresent()) {
      rotationalMagnitude_perc = m_altCommandedRotation_perc.get();
    }

    /* ---------------- Clamping ---------------- */
    translationMagnitude_perc = m_rampRate.calculate(translationMagnitude_perc);
    translationMagnitude_perc = Math.max(-1, Math.min(1, translationMagnitude_perc));
    rotationalMagnitude_perc = Math.max(-1, Math.min(1.0, rotationalMagnitude_perc));

    /* ---------------- Drive ---------------- */
    double yDist = m_isRotateCenter
        ? m_selectedBot.getDistToFront()
        : 0.0;

    m_drive.driveWithGyro(
        translationAngle_deg,
        translationMagnitude_perc,
        rotationalMagnitude_perc,
        0.0,
        yDist);

    m_lastTranslationAngle_deg = translationAngle_deg;
  }


  private double calcThrottle_perc(double stickThrottle_perc) {
    // Convert throttle from [-1,1] to [0,1] with a floor
    double throttle_perc = ((stickThrottle_perc + 1) / 2.0) + 0.1;

    // Slow the bot down if we are aiming at something
    if (m_altCommandedRotation_perc.isPresent()) {
      throttle_perc = throttle_perc / 2.0;
    }

    return throttle_perc;
  }


  // The formula for ellipical input normalization comes from here:
  // https://raw.org/article/how-to-map-a-square-to-a-circle/
  private double calcTranslationMagnitude_perc(double stickX_perc, double stickY_perc) {
    double xPrime_perc = stickX_perc * Math.sqrt(1 - (Math.pow(stickY_perc, 2) / 2));
    double yPrime_perc = stickY_perc * Math.sqrt(1 - (Math.pow(stickX_perc, 2) / 2));

    double magnitude_perc = Math.sqrt(Math.pow(xPrime_perc, 2) + Math.pow(yPrime_perc, 2));
    return magnitude_perc * 2; //TODO This is a gross hack. Figure out the real problem please
  }


  private double calcTranslationAngle_deg(double stickX_perc, double stickY_perc) {
    double translationAngle_deg = Math.toDegrees(Math.atan2(stickX_perc, stickY_perc));
    return (translationAngle_deg + 360.0) % 360.0;
  }


  public void setFrontDrive(boolean isTrue) {
    m_isRotateCenter = isTrue;
  }


  public void setAlternateRotation(Optional<Double> altRotation_perc) {
    m_altCommandedRotation_perc = altRotation_perc;
  }


  public void lockWheels() {
    m_lockdown = true;
  }


  private void stickDebug() {
    SmartDashboard.putNumber("STICK X", m_driveXSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK Y", m_driveYSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK ROT", m_rotateSupplier.getAsDouble());
    SmartDashboard.putNumber("STICK THROTTLE", m_throttleSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.driveWithGyro(m_lastTranslationAngle_deg, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
