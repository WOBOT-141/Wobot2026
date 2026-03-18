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

import frc.robot.Constants.ShooterConstants;

import frc.robot.Util.BumpShooterVector;
import frc.robot.commands.ICommandFactory;
import frc.robot.commands.MovingRotateToTag;

import frc.robot.commands.AutonomousCommands.MoreBasicAutons.DoNothing;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.SitShots;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.DepotLease;

import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftHalfMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightHalfMidwayGather;

import frc.robot.commands.DriveCommands.StickCommand;

import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;

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

  private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.Tags,
      () -> DriverStation.getAlliance());

  private final IBotSelector m_selectedBot = new MainBot();

  private final ChooserHandeling m_autonSelector;

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

    m_stickCommand = new StickCommand(m_drive, m_gyro, m_selectedBot,
        () -> m_flightController0.getDriveX(),
        () -> m_flightController0.getDriveY(),
        () -> m_flightController0.getRotate(),
        () -> m_flightController0.getThrottle());

    m_drive.setDefaultCommand(m_stickCommand);

    configureBindings();

    Map<String, ICommandFactory> autonCommands = Map.ofEntries(

        entry("00 Do Nothing", (ICommandFactory) () -> DoNothing.create()),
        entry("01 Shoot and sit", (ICommandFactory) () -> SitShots.create(m_shooter, m_magazine, m_gyro)),
        entry("02 Depotlease", (ICommandFactory) () -> DepotLease.create(m_drive, m_wheels, m_gyro)),
        entry("03 RIGHT Midway Gather",
            (ICommandFactory) () -> RightMidwayGather.create(m_drive, m_wheels, m_gyro, m_shooter, m_magazine,
                m_intake)),
        entry("04 LEFT Midway Gather",
            (ICommandFactory) () -> LeftMidwayGather.create(m_drive, m_wheels, m_gyro, m_shooter, m_magazine,
                m_intake)),
        entry("05 Right HALF Midway Gather",
            (ICommandFactory) () -> RightHalfMidwayGather.create(m_drive, m_wheels, m_gyro, m_shooter, m_magazine,
                m_intake)),
        entry("06 Left HALF Midway Gather", (ICommandFactory) () -> LeftHalfMidwayGather.create(m_drive, m_wheels,
            m_gyro, m_shooter, m_magazine, m_intake)));

    m_autonSelector = new ChooserHandeling("AutonChooser", autonCommands);
  }

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

  private void configureBindings() {
    configureXboxController();
    configureJoystick();
    configureFlightController();
  }

  private void configureXboxController() {

    m_xboxController0.leftBumper().onTrue(m_intake.intakeExtendCommand());
    m_xboxController0.leftTrigger()
        .whileTrue(m_intake.doubleButtonIntakeRunCommand(() -> m_xboxController0.getRightTriggerAxis()));
    m_xboxController0.rightTrigger().onTrue(m_shooter.homeHoodCommand());
    m_xboxController0.povUp().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchAngleUp()));
    m_xboxController0.povDown().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchAngleDown()));
    m_xboxController0.povLeft().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchSpeedUp()));
    m_xboxController0.povRight().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchSpeedDown()));
    m_xboxController0.a().whileTrue(fireShooterCommand(280, 72));
    m_xboxController0.x().whileTrue(fireShooterCommand(377, 65));
    m_xboxController0.y().whileTrue(fireShooterCommand(335, 65));
    m_xboxController0.b().whileTrue(fireShooterCommand(1000, ShooterConstants.hoodLowerLimit_deg));
    m_xboxController0.back()
        .whileTrue(Commands.startEnd(() -> m_intake.intakeRun(0.8), () -> m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.start()
        .whileTrue(Commands.startEnd(() -> m_magazine.setFeedVelocity(-0.3), () -> m_magazine.cancel(), m_magazine));
    m_xboxController0.leftStick().onTrue(Commands.run(() -> BumpShooterVector.reset()));
  }

  private void configureJoystick() {
    m_joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
    m_joystick.button(2).onTrue(Commands.print("Joystick Button 2"));
  }

  private void configureFlightController() {

    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
        .onTrue(Commands.runOnce(() -> m_gyro.ResetGyro()));
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
        .onTrue(Commands.run(() -> m_stickCommand.lockWheels()));
    new Trigger(() -> m_flightController0.getIButton())
        .whileTrue(new MovingRotateToTag(m_visionServer, m_stickCommand));
  }

  /**
   * Reset swerve modules and apply encoder offsets
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

  private Command autonShooterCommand(double speed, double angle) {

    return Commands.sequence(Commands.run(() -> {
      m_shooter.setHoodAngle(angle);
      m_shooter.setShooterVelocity(speed);
    },
        m_shooter).until(() -> m_shooter.isReadyToShoot()),

        // Feed + keep shooter running at same time
        Commands.deadline(
            Commands.waitSeconds(5),
            // Keep shooter running
            Commands.run(() -> {
              m_shooter.setHoodAngle(angle);
              m_shooter.setShooterVelocity(speed);
            },
                m_shooter),

            // Feed
            Commands.startEnd(() -> m_magazine.setFeedVelocity(0.8), () -> m_magazine.cancel(), m_magazine)),

        // Stop everything
        Commands.runOnce(() -> {
          m_shooter.cancel();
          m_magazine.cancel();
        }));
  }

  private Command fireShooterCommand(double speed, double angle) {

    return Commands.deadline(m_shooter.runShooterCommand(speed, angle),Commands.sequence(Commands.waitUntil(() -> m_shooter.isReadyToShoot()),
            m_magazine.feedCommand()),

        Commands.runOnce(() -> m_drive.lockWheels(), m_drive));
  }

  public Command getAutonomousCommand() {

    Command selectedCommand = m_autonSelector.getSelected();

    if (selectedCommand != null) {
      System.out.println("Selected auton: " + selectedCommand.getName());
      return selectedCommand;
    }

    System.out.println("Selected auton: <null>");
    return Commands.none();
  }

}
