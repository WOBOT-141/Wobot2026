package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.SpektrumFlightController;
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
import frc.robot.Constants.VisionServerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimAtSpecificTag;
import frc.robot.commands.ShooterCalculationAimCommand;
import frc.robot.commands.StopRobot;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.DoNothing;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.IntakeCommands.DoubleButtonIntakeRunCommand;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.IntakeCommands.IntakeJams;
import frc.robot.commands.IntakeCommands.IntakeRetract;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.commands.ShooterCommands.SendHoodToHome;
import frc.robot.commands.ShooterCommands.MagazineAndLock;
import frc.robot.commands.IntakeCommands.RunUntilCurrentLimit;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.commands.ShooterCommands.RotShakes;
import frc.robot.commands.StickParameters.StickBParameters;
import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.TestRightMiday;
import frc.robot.commands.AutonomousCommands.StraightLineDrivePlus;
import frc.robot.commands.AutonomousCommands.MIdwaybutBetter.DoubleRIghtSwipe;
import frc.robot.commands.AutonomousCommands.MIdwaybutBetter.HALFswipeRIGHT;
import frc.robot.commands.AutonomousCommands.MIdwaybutBetter.RRRightMidwayAuton;
import frc.robot.commands.AutonomousCommands.MidwayStuff.LeftHalfMidwayGather;
import frc.robot.commands.AutonomousCommands.MidwayStuff.RightHalfMidwayGather;
import frc.robot.commands.AutonomousCommands.MoreBasicAutons.SitShots;

public class RobotContainer {

  private final SwerveModule[] m_wheels;
  private final Drivetrain m_drive;
  private final IGyro m_gyro = new NavxGyro();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final MagazineAndFeed m_magazine = new MagazineAndFeed();
  // private final Climber m_climbe = new Climber();

  private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0, new StickBParameters());
  private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
  private final CommandJoystick m_joystick = new CommandJoystick(2);
  private final StickCommand m_stickCommand;

  private final VisionServer m_visionServer = new VisionServer(VisionServerConstants.NumberOfCameras, RebuiltAprilTags.Tags.length);
  private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.Tags, () -> DriverStation.getAlliance());

  private final IBotSelector m_selectedBot = new MainBot();
  // private final IBotSelector m_selectedBot = new SummerBot();

 private final SendableChooser<Command> m_autonChooser = new SendableChooser<>();

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

    m_stickCommand = new StickCommand(
        m_drive,
        m_gyro,
        m_selectedBot,
        () -> m_flightController0.getDriveX(),
        () -> m_flightController0.getDriveY(),
        () -> m_flightController0.getRotate(),
        () -> m_flightController0.getThrottle());
        
    m_drive.setDefaultCommand(m_stickCommand);

    configureBindings();

    m_autonChooser.setDefaultOption("00 Do Nothing", new DoNothing());
    m_autonChooser.addOption("01 Shoot and sit",
       new SitShots(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake));
    m_autonChooser.addOption("03 RIGHT Midway Gather",
        new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer, m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("04 LEFT Midway Gather",
        new LeftMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer, m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("05 Right HALF Midway Gather",
        new RightHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer, m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("06 Left HALF Midway Gather",
        new LeftHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake, m_visionServer, m_stickCommand, m_aprilTagFilter, m_xboxController0));
    m_autonChooser.addOption("TEST Rotate to tag",
        new AimAtSpecificTag(11, 27, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0, m_drive));
            SmartDashboard.putData("AutonChooser", m_autonChooser);
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
    m_xboxController0.leftBumper().onTrue(new IntakeExtend(m_intake));
    // m_xboxController0.leftBumper().whileTrue(m_intake.extendIntakeCommand(IntakeConstants.intakeExtendSpeed_perc));
    // m_xboxController0.leftTrigger().whileTrue(Commands.startEnd(() -> m_intake.intakeRun(IntakeConstants.rollerInSpeed_rpm),() -> m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.leftTrigger().whileTrue((new DoubleButtonIntakeRunCommand(m_intake, () -> m_xboxController0.getRightTriggerAxis())));
    m_xboxController0.rightBumper().whileTrue(new MagazineAndLock(m_drive, m_magazine));// locks wheels and runs feed/mag 
    // m_xboxController0.rightBumper().onTrue(new IntakeRetract(m_intake));

    // m_xboxController0.povUp().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchAngleUp())); //Angle up
    // m_xboxController0.povDown().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchAngleDown())); // Angle down
    // m_xboxController0.povLeft().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchSpeedUp())); //speed up
    // m_xboxController0.povRight().onTrue(Commands.runOnce(() -> BumpShooterVector.bumpLaunchSpeedDown())); //speed down
    // m_xboxController0.povLeft().whileTrue(new ShakingBot(0, m_drive, m_magazine)); //Forward back -- Hub
    m_xboxController0.povLeft().whileTrue(m_intake.extendIntakeCommand(IntakeConstants.intakeExtendSpeed_perc));
    // m_xboxController0.povRight().whileTrue(new ShakingBot(45, m_drive, m_magazine)); // Diag Right -- Hub
    // m_xboxController0.povRight().whileTrue(new RunUntilCurrentLimit(-0.1, 30, m_intake));
    m_xboxController0.povRight().whileTrue(new IntakeJams(m_intake));
    // m_xboxController0.povRight().onTrue(new IntakeRetract(m_intake));
    // m_xboxController0.povRight().whileTrue(new ShakingBot(90, m_drive)); //Side side -- Hub/Tower
    m_xboxController0.povDown().whileTrue(new RotShakes(0.3, false, m_drive, m_magazine));// Hub
    // m_xboxController0.povUp().whileTrue(new ShakingBot(135, m_drive, m_magazine)); //Diag Left forward -- Tower
     m_xboxController0.povUp().whileTrue(new RotShakes(0.3, true, m_drive, m_magazine));// Tower

    // m_xboxController0.a().whileTrue(new ShooterCalculationAimCommand(35, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 36, m_xboxController0, m_shooter));
    m_xboxController0.a().whileTrue(new RevShooter( 150, 70, m_shooter, m_xboxController0));//rev hub //previously 180 for launch speed //TODO addvibration last priority split rev and fire 
    m_xboxController0.x().whileTrue(new RevShooter(190, 65, m_shooter, m_xboxController0));//rev bump //used to be 200
    // m_xboxController0.x().onTrue(new IntakeRetract(m_intake));
    m_xboxController0.y().whileTrue(new ShooterCalculationAimCommand(141, ShooterConstants.muzzleToRimHeight_in, ShooterConstants.hubRimToCenter_in, 27, m_xboxController0, m_shooter)); // target difference previously 100, height above rim og = 24
    // m_xboxController0.y().whileTrue(new RevShooter( 330, 58, m_shooter, m_xboxController0));//rev tower SWEETSPOT TEST
    // m_xboxController0.b().whileTrue(new RevShooter(500, 55, m_shooter, m_xboxController0));//rev neutral */
    m_xboxController0.b().whileTrue(Commands.startEnd(() -> m_magazine.setFeedVelocity(-0.3), () -> m_magazine.cancel(), m_magazine));
    
    // m_xboxController0.b().whileTrue(new AimAtHubCommand(m_stickCommand, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0));
    // m_xboxController0.b().whileTrue(new AimAtSpecificTag(10, 26, m_visionServer, m_shooter, m_gyro, m_aprilTagFilter, m_xboxController0, m_drive));

    m_xboxController0.back().whileTrue(Commands.startEnd(() -> m_intake.intakeRun(IntakeConstants.rollerOutSpeed_rpm), () -> m_intake.intakeRollerStop(), m_intake));
    m_xboxController0.start().whileTrue(Commands.startEnd(() -> m_magazine.setFeedVelocity(-0.3), () -> m_magazine.cancel(), m_magazine));
    
    // m_xboxController0.leftStick().onTrue(Commands.run(() -> BumpShooterVector.reset())); // reset angle/speed
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
  }

 public Command getAutonomousCommand() {
    Command selectedCommand = m_autonChooser.getSelected();

    if (selectedCommand != null) {
      System.out.println("Selected auton: " + selectedCommand.getName());
      return selectedCommand;
    }

    System.out.println("Selected auton: <null>");
    return Commands.none();
  }

  }

