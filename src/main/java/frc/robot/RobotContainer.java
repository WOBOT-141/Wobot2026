package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SpektrumFlightController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.MovingRotateToTag;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.Depotlease;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.DoNothing;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.MidwayGather;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.ShootClimbLEFT;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.ShootClimbMIDDLE;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.ShootClimbRIGHT;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.ZeeClimb;
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.ShooterCommands.SetPIDShooter;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;
import frc.robot.subsystems.ChooserHandeling;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.RebuiltAprilTags;


public class RobotContainer {

  private final SwerveModule[] m_wheels;
  private final Drivetrain m_drive;
  private final IGyro m_gyro = new NavxGyro();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final MagazineAndFeed m_magazine = new MagazineAndFeed();

  private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0);
  private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
  private final CommandJoystick m_joystick = new CommandJoystick(2);
  private final StickCommand m_stickCommand;

  private final VisionServer m_visionServer = new VisionServer();
  private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.Tags, () -> DriverStation.getAlliance());

  private final IBotSelector m_selectedBot = new MainBot();
  // private final IBotSelector _selectedBot = new SummerBot();

  private ChooserHandeling m_autonSelector;

  public RobotContainer() {

    m_wheels = new SwerveModule[] {

        new SwerveModule(
            Constants.CANIdConstants.frontLeftSteerMotorID,
            Constants.CANIdConstants.frontLeftDriveMotorID,
            Constants.CANIdConstants.frontLeftEncoderid,
            m_selectedBot.getfrontLeftOffset(),
            m_selectedBot.getfrontLeftLocationX(),
            m_selectedBot.getfrontLeftLocationY(),
            m_selectedBot.rotationsPerInch_rot()),

        new SwerveModule(
            Constants.CANIdConstants.backLeftSteerMotorID,
            Constants.CANIdConstants.backLeftDriveMotorID,
            Constants.CANIdConstants.backLeftEncoderid,
            m_selectedBot.getbackLeftOffset(),
            m_selectedBot.getbackLeftLocationX(),
            m_selectedBot.getbackLeftLocationY(),
            m_selectedBot.rotationsPerInch_rot()),

        new SwerveModule(
            Constants.CANIdConstants.frontRightSteerMotorID,
            Constants.CANIdConstants.frontRightDriveMotorID,
            Constants.CANIdConstants.frontRightEncoderid,
            m_selectedBot.getfrontRightOffset(),
            m_selectedBot.getfrontRightLocationX(),
            m_selectedBot.getfrontRightLocationY(),
            m_selectedBot.rotationsPerInch_rot()),

        new SwerveModule(
            Constants.CANIdConstants.backRightSteerMotorID,
            Constants.CANIdConstants.backRightDriveMotorID,
            Constants.CANIdConstants.backRightEncoderid,
            m_selectedBot.getbackRightOffset(),
            m_selectedBot.getbackRightLocationX(),
            m_selectedBot.getbackRightLocationY(),
            m_selectedBot.rotationsPerInch_rot())
    };

    m_drive = new Drivetrain(m_wheels, m_gyro);

    configureFlightControllerParameters();

    m_stickCommand = new StickCommand(
        m_drive,
        m_gyro,
        m_selectedBot,
        () -> m_flightController0.getDriveX(),
        () -> m_flightController0.getDriveY(),
        () -> m_flightController0.getRotate(),
        () -> m_flightController0.getThrottle(),
        () -> m_flightController0.getSelectPressButton());
        
    m_drive.setDefaultCommand(m_stickCommand);
    
    configureBindings();

    m_autonSelector = new ChooserHandeling("Autonomous Selector",
        Map.ofEntries(
            entry("00 Do Nothing", () -> new DoNothing()),
            entry("01 MIDDLE shooterclimb", () -> new ShootClimbMIDDLE(m_drive, m_wheels, m_gyro)),
            entry("02 LEFT shooterclimb", () -> new ShootClimbLEFT(m_drive, m_wheels, m_gyro)),
            entry("03 RIGHT shooterclimb", () -> new ShootClimbRIGHT(m_drive, m_wheels, m_gyro)),
            entry("04 Depotlease", () -> new Depotlease(m_drive, m_wheels, m_gyro)), // dont use
            entry("05 ZeeClimb", () -> new ZeeClimb(m_drive, m_wheels, m_gyro)),
            entry("06 Midway Gather", () -> new MidwayGather(m_drive, m_wheels, m_gyro)) // fix
        ));
  }

  /**
   * Restore flight controller scaling, offsets, and deadbands.
   */
  private void configureFlightControllerParameters() {

    IStickParameters p = new StickBParameters();

    m_flightController0.setRotateFactor(p.get_RotateFactor());
    m_flightController0.setDriveXFactor(p.get_DriveXFactor());
    m_flightController0.setDriveYFactor(p.get_DriveYFactor());
    m_flightController0.setThrottleFactor(p.get_ThrottleFactor());

    m_flightController0.setRotateOffset(p.get_RotateOffset());
    m_flightController0.setDriveXOffset(p.get_DriveXOffset());
    m_flightController0.setDriveYOffset(p.get_DriveYOffset());
    m_flightController0.setThrottleOffset(p.get_ThrottleOffset());

    m_flightController0.setKnobOffset(p.get_knobOffset());
    m_flightController0.setxDeadBand(p.get_deadbandX());
    m_flightController0.setyDeadBand(p.get_deadbandY());
    m_flightController0.setRotationDeadBand(p.get_deadbandRot());
  }

  /**
   * Re-zero swerve modules while disabled.
   */
  public void wheelSetup() {

    m_wheels[0].setEncoderOffset(m_selectedBot.getfrontLeftOffset());
    m_wheels[0].set_xPos(m_selectedBot.getfrontLeftLocationX());
    m_wheels[0].set_yPos(m_selectedBot.getfrontLeftLocationY());

    m_wheels[1].setEncoderOffset(m_selectedBot.getbackLeftOffset());
    m_wheels[1].set_xPos(m_selectedBot.getbackLeftLocationX());
    m_wheels[1].set_yPos(m_selectedBot.getbackLeftLocationY());

    m_wheels[2].setEncoderOffset(m_selectedBot.getfrontRightOffset());
    m_wheels[2].set_xPos(m_selectedBot.getfrontRightLocationX());
    m_wheels[2].set_yPos(m_selectedBot.getfrontRightLocationY());

    m_wheels[3].setEncoderOffset(m_selectedBot.getbackRightOffset());
    m_wheels[3].set_xPos(m_selectedBot.getbackRightLocationX());
    m_wheels[3].set_yPos(m_selectedBot.getbackRightLocationY());

    for (int i = 0; i < 4; i++) {
      m_wheels[i].settoZero();
    }
  }

  private void configureBindings() {
    configureXboxController();
    configureJoystick();
    configureFlightController();
  }

  private void configureXboxController() {
    m_xboxController0.leftTrigger().whileTrue(Commands.startEnd(() -> m_intake.intakeRun(0.2), () -> m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.leftBumper().whileTrue(Commands.startEnd(() -> m_magazine.setFeedMotorVelocity(0.7), () -> m_magazine.cancel(), m_magazine));
    m_xboxController0.rightBumper().whileTrue(Commands.run(() -> m_intake.intakeUp(), m_intake).until(() -> m_intake.getElevationCurrent() > IntakeConstants.elevationCurrentLimit_amp).finallyDo(() -> m_intake.stopArm()));
    m_xboxController0.rightTrigger().whileTrue(Commands.run(() -> m_intake.intakeDown(), m_intake).until(() -> m_intake.getElevationCurrent() > IntakeConstants.elevationCurrentLimit_amp).finallyDo(() -> m_intake.stopArm()));
    m_xboxController0.y().whileTrue(Commands.startEnd(() -> m_shooter.setShooter_ips(3500), () -> m_shooter.cancel(), m_shooter));
    m_xboxController0.b().whileTrue(Commands.startEnd(() -> m_shooter.setShooter_ips(377), () -> m_shooter.cancel(), m_shooter));
    m_xboxController0.back().whileTrue(Commands.startEnd(() -> m_magazine.setFeedMotorVelocity(-0.5), () -> m_magazine.cancel(), m_magazine));
    m_xboxController0.x().onTrue(new SetPIDShooter(m_shooter));

    // m_xboxController0.rightBumper().whileTrue(new intakeCommand(m_intake, true));
   // m_xboxController0.leftBumper().whileTrue(
     // Commands.startRun( () -> m_shooter.setShooter_ips(100), () -> m_shooter.setShooter_ips(100), m_shooter));

    // m_xboxController0.b().onTrue(new SendHoodToHome(m_shooter));
    // m_xboxController0.a().onTrue(new IntakeHome(m_intake));
    // m_xboxController0.y().onTrue(new IntakeUp(m_intake));
    // m_xboxController0.leftTrigger().onTrue(new IntakeDown(m_intake));
  }

  private void configureJoystick() {
    m_joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
    m_joystick.button(2).onTrue(Commands.print("Joystick Button 2"));
  }

  private void configureFlightController() {
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
        .onTrue(new ResetGyro(m_gyro));
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
        .onTrue(new LockWheels(m_drive));
    new Trigger(() -> m_flightController0.getIButton())
        .whileTrue(new MovingRotateToTag(m_visionServer, m_stickCommand));
  }

  public Command getAutonomousCommand() {

    if (m_autonSelector == null) {
      return Commands.none();
    }

    Command selectedCommand = m_autonSelector.getSelected();

    if (selectedCommand != null) {
      System.out.println("Selected auton: " + selectedCommand);
    } else {
      System.out.println("Selected auton: <null>");
    }

    return selectedCommand;
  }
}
