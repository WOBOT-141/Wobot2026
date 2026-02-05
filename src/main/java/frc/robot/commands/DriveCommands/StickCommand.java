// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IBotSelector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.lib.SpektrumFlightController;

public class StickCommand extends Command {
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drive;
  private final IGyro m_gyro;

  private final SpektrumFlightController m_flightController;
  // private final XboxController m_XboxController;
  private double lastRightDirectional = 0.0;
  private SlewRateLimiter rampRate;
  private boolean isRotateCenter = false;
  private boolean lockdown = false;
  private final IBotSelector _selectedBot;

  /**
   *
   * @param drive
   *          The subsystem used by this command.
   * @param gyro
   *          TODO
   */
  public StickCommand(Drivetrain drive, IGyro gyro, SpektrumFlightController m_flightController, IBotSelector selectedBot) {
    m_drive = drive;
    m_gyro = gyro;
    addRequirements(drive);
    _selectedBot = selectedBot;
    this.m_flightController = m_flightController;
    // m_XboxController = xboxController;
    rampRate = new SlewRateLimiter(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  double stickX;
  double stickY;
  double stickRotate;
  double stickThrottle;
  boolean outsideDeadband;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stickDebug();

    // if(m_XboxController.getRawAxis(Axis.kRightX.value) > .3 
    // || m_XboxController.getRawAxis(Axis.kRightX.value) < -.3  
    // || m_XboxController.getRawAxis(Axis.kRightY.value) > .3 
    // || m_XboxController.getRawAxis(Axis.kRightY.value) < -.3
    // || m_XboxController.getRawAxis(Axis.kLeftX.value) > .3
    // || m_XboxController.getRawAxis(Axis.kLeftX.value) < -.3){
    //   outsideDeadband = true;
    // }else{
    //   outsideDeadband = false;
    // }
    // SmartDashboard.putBoolean("outside dead", outsideDeadband);
    // SmartDashboard.putNumber("XBox - X", m_XboxController.getRawAxis(Axis.kRightX.value));
    // SmartDashboard.putNumber("Xbox - Y", m_XboxController.getRawAxis(Axis.kRightY.value));
    // SmartDashboard.putNumber("XBox - R", m_XboxController.getRawAxis(Axis.kLeftX.value));
    // SmartDashboard.putNumber("Xbox - T", m_XboxController.getRawAxis(Axis.kLeftY.value));

    
    // if(outsideDeadband){
    //   stickX = m_XboxController.getRawAxis(Axis.kRightX.value);
    //   stickY = -m_XboxController.getRawAxis(Axis.kRightY.value);
    //   stickRotate = m_XboxController.getRawAxis(Axis.kLeftX.value);
    //   stickThrottle = -.8; //TODO check this
    // }else{
      stickX = m_flightController.getDriveX();
      stickY = m_flightController.getDriveY();
      stickRotate = m_flightController.getRotate();
      stickThrottle = m_flightController.getThrottle();
    //   stickX = 0;
    //   stickY = 0;
    //   stickRotate = 0;
    //   stickThrottle = 0;
    // }
    // Calculate the magnitudes and directions
    double rightMagniude = Math
        .sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
    double rightDirectional = Math
        .toDegrees(Math.atan2(stickX, stickY));
    double leftDirectional = Math
        .toDegrees(Math.atan2(stickRotate, stickThrottle));
    double throttleValue = (((stickThrottle + 1) / 2) + .1);

    // Clamp Angle
    rightDirectional = (rightDirectional + 360) % 360;
    leftDirectional = (leftDirectional + 360) % 360;

    // subtract gyro from rightDirectional
    if(!outsideDeadband){
      rightDirectional = (rightDirectional - m_gyro.getRealGyroAngle());
    }

    // Magnitude for the movement of the bot on the right stick
    double finalLinearMagnitude = rightMagniude * throttleValue;
    if (finalLinearMagnitude > 1) {
      finalLinearMagnitude = 1;
    } else if (finalLinearMagnitude < -1) {
      finalLinearMagnitude = -1;
    }

    double rampedMagnitude = rampRate.calculate(finalLinearMagnitude);
    double finalRotationalMagnitude = stickRotate * throttleValue * .85;
    if (finalRotationalMagnitude > 1) {
      finalRotationalMagnitude = 1;
    } else if (finalRotationalMagnitude < -1) {
      finalLinearMagnitude = -1;
    }

    if (m_flightController.getSelectPressButton()) {
      lockdown = true;
    }

    // Locks the wheels
    if (lockdown) {
      m_drive.lockWheels();
      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {
        lockdown = false;
        System.out.println("lockdown");
      }
      System.out.println("working");
    } else {
      // TODO add this to its own method maybe like stick debug is?
      // SmartDashboard.putNumber("Final Linear Magnitude", finalLinearMagnitude);
      // SmartDashboard.putNumber("Ramped Magnitude: ", rampedMagnitude);
      // SmartDashboard.putNumber("Final Rotational Magnitude: ",
      // finalRotationalMagnitude);
      // SmartDashboard.putNumber("Right Directional: ", rightDirectional);
      // SmartDashboard.putNumber("Last Directional", lastRightDirectional);
      if (finalLinearMagnitude != 0 || finalRotationalMagnitude != 0) {
        double yDist = isRotateCenter ? _selectedBot.getDistToFront() : 0;

        m_drive.driveSwerve(rightDirectional, rampedMagnitude, finalRotationalMagnitude, 0, yDist);
        lastRightDirectional = rightDirectional;
      } else {
        rampRate.reset(0);
        m_drive.driveSwerve(lastRightDirectional, 0, 0);
      }
    }

  }
=======

    private final Drive m_drive;
    private final IGyro m_gyro;
    private final SpektrumFlightController m_flightController;
    private final IBotSelector m_selectedBot;

    private final SlewRateLimiter speedRamp = new SlewRateLimiter(4.0); // units: fraction per second (tune this)

    private double lastDirectionDeg = 0.0;
    private boolean isRotateAroundFront = false;
    private boolean lockdownMode = false;

    // Dashboard update throttling
    private int dashboardCounter = 0;
    private static final int DASHBOARD_UPDATE_EVERY = 10;

    public StickCommand(
            Drive drive,
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
        speedRamp.reset(0.0);
        lockdownMode = false;
    }

    @Override
    public void execute() {
        // Throttled dashboard updates
        if (++dashboardCounter % DASHBOARD_UPDATE_EVERY == 0) {
            updateDashboard();
        }

        // Read raw controller inputs
        double stickX = m_flightController.getDriveX();      // positive = right
        double stickY = m_flightController.getDriveY();      // positive = forward
        double stickRotate = m_flightController.getRotate(); // positive = CW or CCW? (check your controller)
        double throttleRaw = m_flightController.getThrottle();

        // Throttle scaling: map -1..1 → ~0.1..1.1 (adjust range as needed)
        double throttle = ((throttleRaw + 1.0) / 2.0) * 1.0 + 0.1;
        throttle = Math.min(1.1, Math.max(0.1, throttle));

        // Magnitude & direction (robot-relative)
        double magnitude = Math.hypot(stickX, stickY);
        if (magnitude < 0.08) magnitude = 0.0; // deadband

        double robotRelativeAngleDeg = Math.toDegrees(Math.atan2(stickX, stickY));
        robotRelativeAngleDeg = (robotRelativeAngleDeg + 360.0) % 360.0;

        // Field-oriented: subtract current robot heading
        double headingDeg = m_gyro.getYaw(); // or getRealGyroAngle() if they are different
        double fieldRelativeAngleDeg = robotRelativeAngleDeg - headingDeg;
        fieldRelativeAngleDeg = ((fieldRelativeAngleDeg % 360.0) + 360.0) % 360.0;

        // Apply throttle scaling
        double scaledSpeed = magnitude * throttle;
        double rampedSpeed = speedRamp.calculate(scaledSpeed);
        rampedSpeed = Math.min(1.0, Math.max(0.0, rampedSpeed));

        // Rotation (assuming positive = CCW, adjust sign if needed)
        double scaledRotation = stickRotate * throttle * 0.9; // 90% max rotation speed
        scaledRotation = Math.min(1.0, Math.max(-1.0, scaledRotation));

        // Lockdown / brake mode
        if (m_flightController.getSelectPressButton()) {
            lockdownMode = true;
        }
        if (lockdownMode) {
            m_drive.lockWheels();
            if (magnitude > 0.05 || Math.abs(stickRotate) > 0.05) {
                lockdownMode = false;
            }
            return;
        }

        // Send command to drive subsystem
        if (rampedSpeed > 0.02 || Math.abs(scaledRotation) > 0.02) {
            double rotCenterY = isRotateAroundFront ? m_selectedBot.getDistToFront() : 0.0;

            m_drive.driveSwerve(
                fieldRelativeAngleDeg,          // field-relative direction
                rampedSpeed,                    // normalized speed 0..1
                scaledRotation * 360.0,         // degrees per second (assuming max 360°/s)
                0.0,                       // rotX (center offset)
                rotCenterY                      // rotY (center offset)
            );

            lastDirectionDeg = fieldRelativeAngleDeg;
        } else {
            // No input → hold last direction, zero speed
            speedRamp.reset(0.0);
            m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
        }
    }

    public void setFrontDrive(boolean isTrue) {
        isRotateAroundFront = isTrue;
    }
>>>>>>> Stashed changes

    private void updateDashboard() {
        SmartDashboard.putNumber("Stick X", m_flightController.getDriveX());
        SmartDashboard.putNumber("Stick Y", m_flightController.getDriveY());
        SmartDashboard.putNumber("Stick Rotate", m_flightController.getRotate());
        SmartDashboard.putNumber("Throttle", m_flightController.getThrottle());
        SmartDashboard.putNumber("Gyro Heading (deg)", m_gyro.getYaw());
        SmartDashboard.putNumber("Field Rel Direction (deg)", m_gyro.getYaw()); // change if needed
        SmartDashboard.putBoolean("Lockdown Active", lockdownMode);
    }

<<<<<<< Updated upstream
  public void stickDebug() {
    SmartDashboard.putNumber("STICK X: ", m_flightController.getDriveX());
    SmartDashboard.putNumber("STICK Y: ", m_flightController.getDriveY());
    SmartDashboard.putNumber("STICK ROT: ", m_flightController.getRotate());
    SmartDashboard.putNumber("STICK THROT: ", m_flightController.getThrottle());
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
=======
=======

    private final Drive m_drive;
    private final IGyro m_gyro;
    private final SpektrumFlightController m_flightController;
    private final IBotSelector m_selectedBot;

    private final SlewRateLimiter speedRamp = new SlewRateLimiter(4.0); // units: fraction per second (tune this)

    private double lastDirectionDeg = 0.0;
    private boolean isRotateAroundFront = false;
    private boolean lockdownMode = false;

    // Dashboard update throttling
    private int dashboardCounter = 0;
    private static final int DASHBOARD_UPDATE_EVERY = 10;

    public StickCommand(
            Drive drive,
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
        speedRamp.reset(0.0);
        lockdownMode = false;
    }

    @Override
    public void execute() {
        // Throttled dashboard updates
        if (++dashboardCounter % DASHBOARD_UPDATE_EVERY == 0) {
            updateDashboard();
        }

        // Read raw controller inputs
        double stickX = m_flightController.getDriveX();      // positive = right
        double stickY = m_flightController.getDriveY();      // positive = forward
        double stickRotate = m_flightController.getRotate(); // positive = CW or CCW? (check your controller)
        double throttleRaw = m_flightController.getThrottle();

        // Throttle scaling: map -1..1 → ~0.1..1.1 (adjust range as needed)
        double throttle = ((throttleRaw + 1.0) / 2.0) * 1.0 + 0.1;
        throttle = Math.min(1.1, Math.max(0.1, throttle));

        // Magnitude & direction (robot-relative)
        double magnitude = Math.hypot(stickX, stickY);
        if (magnitude < 0.08) magnitude = 0.0; // deadband

        double robotRelativeAngleDeg = Math.toDegrees(Math.atan2(stickX, stickY));
        robotRelativeAngleDeg = (robotRelativeAngleDeg + 360.0) % 360.0;

        // Field-oriented: subtract current robot heading
        double headingDeg = m_gyro.getYaw(); // or getRealGyroAngle() if they are different
        double fieldRelativeAngleDeg = robotRelativeAngleDeg - headingDeg;
        fieldRelativeAngleDeg = ((fieldRelativeAngleDeg % 360.0) + 360.0) % 360.0;

        // Apply throttle scaling
        double scaledSpeed = magnitude * throttle;
        double rampedSpeed = speedRamp.calculate(scaledSpeed);
        rampedSpeed = Math.min(1.0, Math.max(0.0, rampedSpeed));

        // Rotation (assuming positive = CCW, adjust sign if needed)
        double scaledRotation = stickRotate * throttle * 0.9; // 90% max rotation speed
        scaledRotation = Math.min(1.0, Math.max(-1.0, scaledRotation));

        // Lockdown / brake mode
        if (m_flightController.getSelectPressButton()) {
            lockdownMode = true;
        }
        if (lockdownMode) {
            m_drive.lockWheels();
            if (magnitude > 0.05 || Math.abs(stickRotate) > 0.05) {
                lockdownMode = false;
            }
            return;
        }

        // Send command to drive subsystem
        if (rampedSpeed > 0.02 || Math.abs(scaledRotation) > 0.02) {
            double rotCenterY = isRotateAroundFront ? m_selectedBot.getDistToFront() : 0.0;

            m_drive.driveSwerve(
                fieldRelativeAngleDeg,          // field-relative direction
                rampedSpeed,                    // normalized speed 0..1
                scaledRotation * 360.0,         // degrees per second (assuming max 360°/s)
                0.0,                       // rotX (center offset)
                rotCenterY                      // rotY (center offset)
            );

            lastDirectionDeg = fieldRelativeAngleDeg;
        } else {
            // No input → hold last direction, zero speed
            speedRamp.reset(0.0);
            m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
        }
    }

    public void setFrontDrive(boolean isTrue) {
        isRotateAroundFront = isTrue;
    }

=======

    private final Drive m_drive;
    private final IGyro m_gyro;
    private final SpektrumFlightController m_flightController;
    private final IBotSelector m_selectedBot;

    private final SlewRateLimiter speedRamp = new SlewRateLimiter(4.0); // units: fraction per second (tune this)

    private double lastDirectionDeg = 0.0;
    private boolean isRotateAroundFront = false;
    private boolean lockdownMode = false;

    // Dashboard update throttling
    private int dashboardCounter = 0;
    private static final int DASHBOARD_UPDATE_EVERY = 10;

    public StickCommand(
            Drive drive,
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
        speedRamp.reset(0.0);
        lockdownMode = false;
    }

    @Override
    public void execute() {
        // Throttled dashboard updates
        if (++dashboardCounter % DASHBOARD_UPDATE_EVERY == 0) {
            updateDashboard();
        }

        // Read raw controller inputs
        double stickX = m_flightController.getDriveX();      // positive = right
        double stickY = m_flightController.getDriveY();      // positive = forward
        double stickRotate = m_flightController.getRotate(); // positive = CW or CCW? (check your controller)
        double throttleRaw = m_flightController.getThrottle();

        // Throttle scaling: map -1..1 → ~0.1..1.1 (adjust range as needed)
        double throttle = ((throttleRaw + 1.0) / 2.0) * 1.0 + 0.1;
        throttle = Math.min(1.1, Math.max(0.1, throttle));

        // Magnitude & direction (robot-relative)
        double magnitude = Math.hypot(stickX, stickY);
        if (magnitude < 0.08) magnitude = 0.0; // deadband

        double robotRelativeAngleDeg = Math.toDegrees(Math.atan2(stickX, stickY));
        robotRelativeAngleDeg = (robotRelativeAngleDeg + 360.0) % 360.0;

        // Field-oriented: subtract current robot heading
        double headingDeg = m_gyro.getYaw(); // or getRealGyroAngle() if they are different
        double fieldRelativeAngleDeg = robotRelativeAngleDeg - headingDeg;
        fieldRelativeAngleDeg = ((fieldRelativeAngleDeg % 360.0) + 360.0) % 360.0;

        // Apply throttle scaling
        double scaledSpeed = magnitude * throttle;
        double rampedSpeed = speedRamp.calculate(scaledSpeed);
        rampedSpeed = Math.min(1.0, Math.max(0.0, rampedSpeed));

        // Rotation (assuming positive = CCW, adjust sign if needed)
        double scaledRotation = stickRotate * throttle * 0.9; // 90% max rotation speed
        scaledRotation = Math.min(1.0, Math.max(-1.0, scaledRotation));

        // Lockdown / brake mode
        if (m_flightController.getSelectPressButton()) {
            lockdownMode = true;
        }
        if (lockdownMode) {
            m_drive.lockWheels();
            if (magnitude > 0.05 || Math.abs(stickRotate) > 0.05) {
                lockdownMode = false;
            }
            return;
        }

        // Send command to drive subsystem
        if (rampedSpeed > 0.02 || Math.abs(scaledRotation) > 0.02) {
            double rotCenterY = isRotateAroundFront ? m_selectedBot.getDistToFront() : 0.0;

            m_drive.driveSwerve(
                fieldRelativeAngleDeg,          // field-relative direction
                rampedSpeed,                    // normalized speed 0..1
                scaledRotation * 360.0,         // degrees per second (assuming max 360°/s)
                0.0,                       // rotX (center offset)
                rotCenterY                      // rotY (center offset)
            );

            lastDirectionDeg = fieldRelativeAngleDeg;
        } else {
            // No input → hold last direction, zero speed
            speedRamp.reset(0.0);
            m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
        }
    }

    public void setFrontDrive(boolean isTrue) {
        isRotateAroundFront = isTrue;
    }

>>>>>>> Stashed changes
=======

    private final Drive m_drive;
    private final IGyro m_gyro;
    private final SpektrumFlightController m_flightController;
    private final IBotSelector m_selectedBot;

    private final SlewRateLimiter speedRamp = new SlewRateLimiter(4.0); // units: fraction per second (tune this)

    private double lastDirectionDeg = 0.0;
    private boolean isRotateAroundFront = false;
    private boolean lockdownMode = false;

    // Dashboard update throttling
    private int dashboardCounter = 0;
    private static final int DASHBOARD_UPDATE_EVERY = 10;

    public StickCommand(
            Drive drive,
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
        speedRamp.reset(0.0);
        lockdownMode = false;
    }

    @Override
    public void execute() {
        // Throttled dashboard updates
        if (++dashboardCounter % DASHBOARD_UPDATE_EVERY == 0) {
            updateDashboard();
        }

        // Read raw controller inputs
        double stickX = m_flightController.getDriveX();      // positive = right
        double stickY = m_flightController.getDriveY();      // positive = forward
        double stickRotate = m_flightController.getRotate(); // positive = CW or CCW? (check your controller)
        double throttleRaw = m_flightController.getThrottle();

        // Throttle scaling: map -1..1 → ~0.1..1.1 (adjust range as needed)
        double throttle = ((throttleRaw + 1.0) / 2.0) * 1.0 + 0.1;
        throttle = Math.min(1.1, Math.max(0.1, throttle));

        // Magnitude & direction (robot-relative)
        double magnitude = Math.hypot(stickX, stickY);
        if (magnitude < 0.08) magnitude = 0.0; // deadband

        double robotRelativeAngleDeg = Math.toDegrees(Math.atan2(stickX, stickY));
        robotRelativeAngleDeg = (robotRelativeAngleDeg + 360.0) % 360.0;

        // Field-oriented: subtract current robot heading
        double headingDeg = m_gyro.getYaw(); // or getRealGyroAngle() if they are different
        double fieldRelativeAngleDeg = robotRelativeAngleDeg - headingDeg;
        fieldRelativeAngleDeg = ((fieldRelativeAngleDeg % 360.0) + 360.0) % 360.0;

        // Apply throttle scaling
        double scaledSpeed = magnitude * throttle;
        double rampedSpeed = speedRamp.calculate(scaledSpeed);
        rampedSpeed = Math.min(1.0, Math.max(0.0, rampedSpeed));

        // Rotation (assuming positive = CCW, adjust sign if needed)
        double scaledRotation = stickRotate * throttle * 0.9; // 90% max rotation speed
        scaledRotation = Math.min(1.0, Math.max(-1.0, scaledRotation));

        // Lockdown / brake mode
        if (m_flightController.getSelectPressButton()) {
            lockdownMode = true;
        }
        if (lockdownMode) {
            m_drive.lockWheels();
            if (magnitude > 0.05 || Math.abs(stickRotate) > 0.05) {
                lockdownMode = false;
            }
            return;
        }

        // Send command to drive subsystem
        if (rampedSpeed > 0.02 || Math.abs(scaledRotation) > 0.02) {
            double rotCenterY = isRotateAroundFront ? m_selectedBot.getDistToFront() : 0.0;

            m_drive.driveSwerve(
                fieldRelativeAngleDeg,          // field-relative direction
                rampedSpeed,                    // normalized speed 0..1
                scaledRotation * 360.0,         // degrees per second (assuming max 360°/s)
                0.0,                       // rotX (center offset)
                rotCenterY                      // rotY (center offset)
            );

            lastDirectionDeg = fieldRelativeAngleDeg;
        } else {
            // No input → hold last direction, zero speed
            speedRamp.reset(0.0);
            m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
        }
    }

    public void setFrontDrive(boolean isTrue) {
        isRotateAroundFront = isTrue;
    }

>>>>>>> Stashed changes
=======

    private final Drive m_drive;
    private final IGyro m_gyro;
    private final SpektrumFlightController m_flightController;
    private final IBotSelector m_selectedBot;

    private final SlewRateLimiter speedRamp = new SlewRateLimiter(4.0); // units: fraction per second (tune this)

    private double lastDirectionDeg = 0.0;
    private boolean isRotateAroundFront = false;
    private boolean lockdownMode = false;

    // Dashboard update throttling
    private int dashboardCounter = 0;
    private static final int DASHBOARD_UPDATE_EVERY = 10;

    public StickCommand(
            Drive drive,
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
        speedRamp.reset(0.0);
        lockdownMode = false;
    }

    @Override
    public void execute() {
        // Throttled dashboard updates
        if (++dashboardCounter % DASHBOARD_UPDATE_EVERY == 0) {
            updateDashboard();
        }

        // Read raw controller inputs
        double stickX = m_flightController.getDriveX();      // positive = right
        double stickY = m_flightController.getDriveY();      // positive = forward
        double stickRotate = m_flightController.getRotate(); // positive = CW or CCW? (check your controller)
        double throttleRaw = m_flightController.getThrottle();

        // Throttle scaling: map -1..1 → ~0.1..1.1 (adjust range as needed)
        double throttle = ((throttleRaw + 1.0) / 2.0) * 1.0 + 0.1;
        throttle = Math.min(1.1, Math.max(0.1, throttle));

        // Magnitude & direction (robot-relative)
        double magnitude = Math.hypot(stickX, stickY);
        if (magnitude < 0.08) magnitude = 0.0; // deadband

        double robotRelativeAngleDeg = Math.toDegrees(Math.atan2(stickX, stickY));
        robotRelativeAngleDeg = (robotRelativeAngleDeg + 360.0) % 360.0;

        // Field-oriented: subtract current robot heading
        double headingDeg = m_gyro.getYaw(); // or getRealGyroAngle() if they are different
        double fieldRelativeAngleDeg = robotRelativeAngleDeg - headingDeg;
        fieldRelativeAngleDeg = ((fieldRelativeAngleDeg % 360.0) + 360.0) % 360.0;

        // Apply throttle scaling
        double scaledSpeed = magnitude * throttle;
        double rampedSpeed = speedRamp.calculate(scaledSpeed);
        rampedSpeed = Math.min(1.0, Math.max(0.0, rampedSpeed));

        // Rotation (assuming positive = CCW, adjust sign if needed)
        double scaledRotation = stickRotate * throttle * 0.9; // 90% max rotation speed
        scaledRotation = Math.min(1.0, Math.max(-1.0, scaledRotation));

        // Lockdown / brake mode
        if (m_flightController.getSelectPressButton()) {
            lockdownMode = true;
        }
        if (lockdownMode) {
            m_drive.lockWheels();
            if (magnitude > 0.05 || Math.abs(stickRotate) > 0.05) {
                lockdownMode = false;
            }
            return;
        }

        // Send command to drive subsystem
        if (rampedSpeed > 0.02 || Math.abs(scaledRotation) > 0.02) {
            double rotCenterY = isRotateAroundFront ? m_selectedBot.getDistToFront() : 0.0;

            m_drive.driveSwerve(
                fieldRelativeAngleDeg,          // field-relative direction
                rampedSpeed,                    // normalized speed 0..1
                scaledRotation * 360.0,         // degrees per second (assuming max 360°/s)
                0.0,                       // rotX (center offset)
                rotCenterY                      // rotY (center offset)
            );

            lastDirectionDeg = fieldRelativeAngleDeg;
        } else {
            // No input → hold last direction, zero speed
            speedRamp.reset(0.0);
            m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
        }
    }

    public void setFrontDrive(boolean isTrue) {
        isRotateAroundFront = isTrue;
    }

>>>>>>> Stashed changes
    private void updateDashboard() {
        SmartDashboard.putNumber("Stick X", m_flightController.getDriveX());
        SmartDashboard.putNumber("Stick Y", m_flightController.getDriveY());
        SmartDashboard.putNumber("Stick Rotate", m_flightController.getRotate());
        SmartDashboard.putNumber("Throttle", m_flightController.getThrottle());
        SmartDashboard.putNumber("Gyro Heading (deg)", m_gyro.getYaw());
        SmartDashboard.putNumber("Field Rel Direction (deg)", m_gyro.getYaw()); // change if needed
        SmartDashboard.putBoolean("Lockdown Active", lockdownMode);
    }

<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    @Override
    public void end(boolean interrupted) {
        m_drive.driveSwerve(lastDirectionDeg, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
}