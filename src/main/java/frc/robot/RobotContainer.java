// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import java.io.File;
import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SpektrumFlightController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionServerConstants;
import frc.robot.commands.AimAtSpecificTag;
import frc.robot.commands.ShooterCalculationAimCommand;
import frc.robot.commands.StopRobot;
import frc.robot.commands.AutonomousCommands.setIntake;
import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftHalfMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightHalfMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightMidwayGather;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.DoNothing;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.SitShots;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.IntakeCommands.DoubleButtonIntakeRunCommand;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.IntakeCommands.IntakeRetract;
import frc.robot.commands.IntakeCommands.RunUntilCurrentLimit;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.commands.ShooterCommands.MagazineAndLock;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.commands.ShooterCommands.RotShakes;
import frc.robot.commands.ShooterCommands.SendHoodToHome;
import frc.robot.commands.ShooterCommands.aimByOdometry;
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
  // private final Climber m_climbe = new Climber();

  private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0);
  private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
  // private final CommandJoystick m_joystick = new CommandJoystick(2);
  private final StickCommand m_stickCommand;

  private final VisionServer m_visionServer = new VisionServer(VisionServerConstants.NumberOfCameras,
      RebuiltAprilTags.Tags.length);
  private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.Tags,
      () -> DriverStation.getAlliance());

  private final IBotSelector m_selectedBot = new MainBot();
  // private final IBotSelector m_selectedBot = new SummerBot();

  private final SendableChooser<Command> m_autonChooser = new SendableChooser<>();
  private LoggedDashboardChooser<Command> autoChooser;
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

        m_drive = new Drivetrain(m_wheels, m_gyro, m_visionServer);

        m_stickCommand = new StickCommand(
                m_drive,
                m_gyro,
                m_selectedBot,
                () -> m_flightController0.getDriveX(),
                () -> m_flightController0.getDriveY(),
                () -> m_flightController0.getRotate(),
                () -> m_flightController0.getThrottle());

    m_autonChooser.setDefaultOption("00 Do Nothing", new DoNothing());
    m_autonChooser.addOption("01 Shoot and sit", new SitShots(m_shooter, m_magazine, m_gyro));
    m_autonChooser.addOption("03 RIGHT Midway Gather",
        new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("04 LEFT Midway Gather",
        new LeftMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer, m_stickCommand,
            m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("05 Right HALF Midway Gather",
        new RightHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("06 Left HALF Midway Gather",
        new LeftHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("TEST Rotate to tag",
        new AimAtSpecificTag(11, 27, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0, m_drive));
    SmartDashboard.putData("AutonChooser", m_autonChooser);
    autoChooser = buildAutoChooserForMode(true);

        configureBindings();

  }

  /**
   * Restore flight controller scaling, offsets, and deadbands.
   */
  private void configureFlightControllerParameters() {
    m_drive.setDefaultCommand(m_stickCommand);

    // Legacy manual chooser handling from HEAD (preserved for reference)
    // m_autonSelector = new ChooserHandeling("Autonomous Selector",
    // Map.ofEntries(
    // entry("00 Do Nothing", () -> new DoNothing()),
    // entry("01 Shoot and sit", () -> new SitShots(m_shooter, m_magazine, m_gyro)),
    // entry("02 Depotlease", () -> new Depotlease(m_drive, m_wheels, m_gyro,
    // m_intake, m_shooter, m_magazine)),
    // entry("03 RIGHT Midway Gather", () -> new RightMidwayGather(m_drive,
    // m_wheels, m_gyro, m_shooter, m_magazine, m_intake)),
    // entry("04 LEFT Midway Gather", () -> new LeftMidwayGather(m_drive, m_wheels,
    // m_gyro, m_shooter, m_magazine, m_intake)),
    // entry("05 Right HALF Midway Gather", () -> new RightHalfMidwayGather(m_drive,
    // m_wheels, m_gyro, m_shooter, m_magazine, m_intake)),
    // entry("05 Left HALF Midway Gather", () -> new LeftHalfMidwayGather(m_drive,
    // m_wheels, m_gyro, m_shooter, m_magazine, m_intake))
    // ));

    // Populating AdvantageKit Logged Chooser with routines from main
    m_autonChooser.setDefaultOption("00 Do Nothing", new DoNothing());
    m_autonChooser.addOption("01 Shoot and sit", new SitShots(m_shooter, m_magazine, m_gyro));
    m_autonChooser.addOption("03 RIGHT Midway Gather",
        new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("04 LEFT Midway Gather",
        new LeftMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("05 Right HALF Midway Gather",
        new RightHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("06 Left HALF Midway Gather",
        new LeftHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer,
            m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("TEST Rotate to tag",
        new AimAtSpecificTag(11, 27, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0,
            m_drive));
  }

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
  /**
   * Restore flight controller scaling, offsets, and deadbands.
   */

   public void configureBindings(){
    configureFlightController();
    configureJoystick();
    configureXboxController();
   }

  private void configureXboxController() {
    m_xboxController0.leftBumper().onTrue(new IntakeExtend(m_intake));
    // m_xboxController0.leftBumper().whileTrue(m_intake.extendIntakeCommand(IntakeConstants.intakeExtendSpeed_perc));
    // m_xboxController0.leftTrigger().whileTrue(Commands.startEnd(() ->
    // m_intake.intakeRun(IntakeConstants.rollerInSpeed_rpm),() ->
    // m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.leftTrigger()
        .whileTrue((new DoubleButtonIntakeRunCommand(m_intake, () -> m_xboxController0.getRightTriggerAxis())));
    m_xboxController0.rightBumper().whileTrue(new MagazineAndLock(m_drive, m_magazine));// locks wheels and runs
                                                                                        // feed/mag
    // m_xboxController0.rightBumper().onTrue(new IntakeRetract(m_intake));

    // m_xboxController0.povUp().onTrue(Commands.runOnce(() ->
    // BumpShooterVector.bumpLaunchAngleUp())); //Angle up
    // m_xboxController0.povDown().onTrue(Commands.runOnce(() ->
    // BumpShooterVector.bumpLaunchAngleDown())); // Angle down
    // m_xboxController0.povLeft().onTrue(Commands.runOnce(() ->
    // BumpShooterVector.bumpLaunchSpeedUp())); //speed up
    // m_xboxController0.povRight().onTrue(Commands.runOnce(() ->
    // BumpShooterVector.bumpLaunchSpeedDown())); //speed down
    // m_xboxController0.povLeft().whileTrue(new ShakingBot(0, m_drive,
    // m_magazine)); //Forward back -- Hub
    m_xboxController0.povLeft().whileTrue(m_intake.extendIntakeCommand(IntakeConstants.intakeExtendSpeed_perc));
    // m_xboxController0.povRight().whileTrue(new ShakingBot(45, m_drive,
    // m_magazine)); // Diag Right -- Hub
    m_xboxController0.povRight().whileTrue(new RunUntilCurrentLimit(-0.1, 30, m_intake));
    // m_xboxController0.povRight().whileTrue(new ShakingBot(90, m_drive)); //Side
    // side -- Hub/Tower
    m_xboxController0.povDown().whileTrue(new RotShakes(0.3, false, m_drive, m_magazine));// Hub
    // m_xboxController0.povUp().whileTrue(new ShakingBot(135, m_drive,
    // m_magazine)); //Diag Left forward -- Tower
    m_xboxController0.povUp().whileTrue(new RotShakes(0.3, true, m_drive, m_magazine));// Tower

    // m_xboxController0.a().whileTrue(new ShooterCalculationAimCommand(35,
    // ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in,
    // 36, m_xboxController0, m_shooter));
    m_xboxController0.a().whileTrue(new RevShooter(180, 70, m_shooter, m_xboxController0));// rev hub //TODO
                                                                                           // addvibration last priority
                                                                                           // split rev and fire
    m_xboxController0.x().whileTrue(new RevShooter(200, 65, m_shooter, m_xboxController0));// rev bump
    // m_xboxController0.x().onTrue(new IntakeRetract(m_intake));
    m_xboxController0.y().whileTrue(new ShooterCalculationAimCommand(100, ShooterConstants.muzzleToRimHeight_in,
        ShooterConstants.hubRimToCenter_in, 24, m_xboxController0, m_shooter));
    // m_xboxController0.y().whileTrue(new RevShooter( 330, 58, m_shooter,
    // m_xboxController0));//rev tower SWEETSPOT TEST
    // m_xboxController0.b().whileTrue(new FireShooter( 1000,
    // ShooterConstants.hoodLowerLimit_deg, m_shooter, m_drive, m_magazine));//rev
    // neutral */
    // m_xboxController0.b().whileTrue(new AimAtHubCommand(m_stickCommand,
    // m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0));
    // m_xboxController0.b().whileTrue(
    //     new AimAtSpecificTag(10, 26, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0, m_drive));
    m_xboxController0.b().whileTrue(new aimByOdometry(m_drive, m_shooter, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24)); //TODO TEST

    m_xboxController0.back().whileTrue(Commands.startEnd(() -> m_intake.intakeRun(IntakeConstants.rollerOutSpeed_rpm),
        () -> m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.start()
        .whileTrue(Commands.startEnd(() -> m_magazine.setFeedVelocity(-0.3), () -> m_magazine.cancel(), m_magazine));

    // m_xboxController0.leftStick().onTrue(Commands.run(() ->
    // BumpShooterVector.reset())); // reset angle/speed
    m_xboxController0.leftStick().onTrue(new SendHoodToHome(m_shooter));
    m_xboxController0.rightStick().onTrue(new StopRobot(m_intake, m_magazine, m_drive, m_shooter));
  }

  private void configureJoystick() {
    // m_joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
    // m_joystick.button(2).onTrue(Commands.print("Joystick Button 2"));

  }

  private void configureFlightController() {
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
        .onTrue(new ResetGyro(m_gyro));
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
        .onTrue(Commands.runOnce(() -> m_stickCommand.lockWheels()));
    // new Trigger(() -> m_flightController0.getIButton())
    // .whileTrue(m_intake.extendIntakeCommand(-IntakeConstants.intakeExtendSpeed_perc));
    // .whileTrue(new AimAtHubCommand(m_stickCommand, m_visionServer, m_shooter,
    // m_gyro, m_aprilTagFilter, m_xboxController0));
    // .whileTrue(new MovingRotateToTag(m_visionServer, m_stickCommand));
  }

  private final Set<String> compAutos = loadCompAutoNames();

  private static Set<String> loadCompAutoNames() {
    Set<String> names = new HashSet<>();
    File autosDir = new File(edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), "pathplanner/autos");
    File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
    if (autoFiles == null)
      return names;

    for (File file : autoFiles) {
      // Auto name = filename without .auto extension
      String autoName = file.getName().replace(".auto", "");
      names.add(autoName);
    }
    return names;
  }

  private LoggedDashboardChooser<Command> buildAutoChooserForMode(boolean competitionMode) {
    SendableChooser<Command> sendable = new SendableChooser<>();
    sendable.setDefaultOption("None", Commands.none());
    for (String autoName : compAutos) {
      sendable.addOption(autoName, new PathPlannerAuto(autoName));
    }
    return new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

  }

  public void pathPlannerCommands(Intake intake, Shooter shooter, MagazineAndFeed magazine) {
    // 1. Initialize subsystems

    // 2. Register named commands
    NamedCommands.registerCommand("ExtendIntake", new IntakeExtend(m_intake));
    NamedCommands.registerCommand("Intake", new setIntake(m_intake, .7));
    NamedCommands.registerCommand("Shoot", new AutonShooter(shooter, magazine, 200, 65));
    NamedCommands.registerCommand("RetractIntake", new IntakeRetract(m_intake));
    NamedCommands.registerCommand("TestShot", new aimByOdometry(m_drive, m_shooter, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 24));

  }

  public Command getAutonomousCommand() {
    Command selectedAuto = autoChooser.get();
    if (selectedAuto == null)
      return null;

    // Only wrap comp autos with shooting setup/teardown; test autos run bare
    String selectedName = autoChooser.getSendableChooser().getSelected();
    if (selectedName == null || !compAutos.contains(selectedName)) {
      return selectedAuto;
    }

    // Reset odometry to the auto's starting pose BEFORE any shooting commands run.
    // Without this, the initial SmartLaunch uses a stale pose (e.g. heading=0 from
    // gyro reset) instead of the auto's actual starting heading, causing the turret 
    // to aim in the wrong direction.
    edu.wpi.first.math.geometry.Pose2d autoStartingPose = null;
    try {
      PathPlannerAuto ppAuto = new PathPlannerAuto(selectedName);
      autoStartingPose = ppAuto.getStartingPose();
      if (autoStartingPose != null) {
        edu.wpi.first.wpilibj.DriverStation.Alliance alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
            .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
          autoStartingPose = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(autoStartingPose);
        }
      }
    } catch (Exception e) {
      // Auto doesn't have a valid starting pose - will skip reset
    }
    final edu.wpi.first.math.geometry.Pose2d startingPose = autoStartingPose;
    return selectedAuto;
  }
public class DriveSubsystem extends SubsystemBase {
  public DriveSubsystem() {
    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config; // this is in settings.json
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

  }
}

}
