package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // Added PS4 Import
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SpektrumFlightController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionServerConstants; // Added for hood limits
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;
import frc.robot.subsystems.ChooserHandeling;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.GyroIO.GyroIO;
import frc.robot.subsystems.GyroIO.GyroIORealNavX;
import frc.robot.subsystems.GyroIO.GyroIOSim;
import frc.robot.subsystems.SwerveModuleIO.ModuleIOReal;
import frc.robot.subsystems.SwerveModuleIO.ModuleIOSim;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.RebuiltAprilTags;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.shooterIO.ShooterIOReal;
import frc.robot.subsystems.shooterIO.ShooterIOSim;

public class RobotContainer {

    private final SwerveModule[] m_wheels;
    private final Drivetrain m_drive;
    private IGyro m_gyro;

    private final Shooter m_shooter;

    // private final Intake m_intake = new Intake();
    // private final MagazineAndFeed m_magazine = new MagazineAndFeed();
    // private final Climber m_climbe = new Climber();
    private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0);
    private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
    private final CommandJoystick m_joystick = new CommandJoystick(2);

    private final CommandPS4Controller m_ps4Controller = new CommandPS4Controller(3);

    private final StickCommand m_stickCommand;

    private final VisionServer m_visionServer = new VisionServer(VisionServerConstants.NumberOfCameras, RebuiltAprilTags.Tags.length);
    private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.Tags, () -> DriverStation.getAlliance());

    // private final IBotSelector m_selectedBot = new MainBot();
    private final IBotSelector m_selectedBot = new SummerBot();

    private ChooserHandeling m_autonSelector;

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    public RobotContainer() {

        if (RobotBase.isReal()) {
            m_shooter = new Shooter(new ShooterIOReal());
            m_gyro = new Gyro(new GyroIORealNavX());
        } else {
            m_shooter = new Shooter(new ShooterIOSim());
            m_gyro = new Gyro(new GyroIOSim());

        }

        m_wheels = new SwerveModule[]{
            new SwerveModule(
            RobotBase.isReal()
            ? new ModuleIOReal(
            Constants.CANIdConstants.frontLeftSteerMotorID,
            Constants.CANIdConstants.frontLeftDriveMotorID,
            Constants.CANIdConstants.frontLeftEncoderid,
            m_selectedBot.rotationsPerInch_rot())
            : new ModuleIOSim(),
            Constants.CANIdConstants.frontLeftSteerMotorID,
            Constants.CANIdConstants.frontLeftDriveMotorID,
            m_selectedBot.getfrontLeftOffset(),
            m_selectedBot.getfrontLeftLocationX(),
            m_selectedBot.getfrontLeftLocationY()),
            new SwerveModule(
            RobotBase.isReal()
            ? new ModuleIOReal(
            Constants.CANIdConstants.backLeftSteerMotorID,
            Constants.CANIdConstants.backLeftDriveMotorID,
            Constants.CANIdConstants.backLeftEncoderid,
            m_selectedBot.rotationsPerInch_rot())
            : new ModuleIOSim(),
            Constants.CANIdConstants.backLeftSteerMotorID,
            Constants.CANIdConstants.backLeftDriveMotorID,
            m_selectedBot.getbackLeftOffset(),
            m_selectedBot.getbackLeftLocationX(),
            m_selectedBot.getbackLeftLocationY()),
            new SwerveModule(
            RobotBase.isReal()
            ? new ModuleIOReal(
            Constants.CANIdConstants.frontRightSteerMotorID,
            Constants.CANIdConstants.frontRightDriveMotorID,
            Constants.CANIdConstants.frontRightEncoderid,
            m_selectedBot.rotationsPerInch_rot())
            : new ModuleIOSim(),
            Constants.CANIdConstants.frontRightSteerMotorID,
            Constants.CANIdConstants.frontRightDriveMotorID,
            m_selectedBot.getfrontRightOffset(),
            m_selectedBot.getfrontRightLocationX(),
            m_selectedBot.getfrontRightLocationY()),
            new SwerveModule(
            RobotBase.isReal()
            ? new ModuleIOReal(
            Constants.CANIdConstants.backRightSteerMotorID,
            Constants.CANIdConstants.backRightDriveMotorID,
            Constants.CANIdConstants.backRightEncoderid,
            m_selectedBot.rotationsPerInch_rot())
            : new ModuleIOSim(),
            Constants.CANIdConstants.backRightSteerMotorID,
            Constants.CANIdConstants.backRightDriveMotorID,
            m_selectedBot.getbackRightOffset(),
            m_selectedBot.getbackRightLocationX(),
            m_selectedBot.getbackRightLocationY())
        };

        m_drive = new Drivetrain(m_wheels, m_gyro, m_visionServer);

        configureFlightControllerParameters();

        m_stickCommand = new StickCommand(
                m_drive,
                m_gyro,
                m_selectedBot,
                () -> m_flightController0.getDriveX(),
                () -> m_flightController0.getDriveY(),
                () -> m_flightController0.getRotate(),
                () -> m_flightController0.getThrottle());

        // m_drive.setDefaultCommand(m_stickCommand);

        configureBindings();

        // m_autonSelector = new ChooserHandeling("Autonomous Selector",
        //     Map.ofEntries(
        //         entry("00 Do Nothing", () -> new DoNothing()),
        //         entry("01 Shoot and sit", () -> new SitShots(m_shooter, m_magazine, m_gyro)),
        //         entry("02 Depotlease", () -> new Depotlease(m_drive, m_wheels, m_gyro, m_intake, m_shooter, m_magazine)),
        //         entry("03 RIGHT Midway Gather", () -> new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake)),
        //         entry("04 LEFT Midway Gather", () -> new LeftMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake)),
        //         entry("05 Right HALF Midway Gather", () -> new RightHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake)),
        //         entry("05 Left HALF Midway Gather", () -> new LeftHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake))
        //         //Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake
        //          // fix
        //     ));
        // autoChooser.addOption("00 Do Nothing", new DoNothing());
        // autoChooser.addDefaultOption("01 Shoot and sit", new SitShots(m_shooter, m_magazine, m_gyro));
        // autoChooser.addOption("02 Depotlease", new Depotlease(m_drive, m_wheels, m_gyro, m_intake, m_shooter, m_magazine));
        // autoChooser.addOption("03 RIGHT Midway Gather", new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake));    
        // autoChooser.addOption("04 LEFT Midway Gather", new LeftMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake));
        // autoChooser.addOption("05 Right HALF Midway Gather", new RightHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake));
        // autoChooser.addOption("05 Left HALF Midway Gather", new LeftHalfMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake));
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
        configurePS4Controller(); // Hooked up the PS4 bindings here
    }

    private void configureXboxController() {
        //m_xboxController0.leftBumper().onTrue(new IntakeExtend(m_intake));
        //m_xboxController0.leftTrigger().whileTrue(new DoubleButtonIntakeRunCommand(m_intake, () -> m_xboxController0.getRightTriggerAxis()));
        //m_xboxController0.rightBumper().whileTrue(new MagazineAndLock(m_drive, m_magazine));// locks wheels and runs feed/mag 
        //m_xboxController0.rightBumper().whileTrue(Commands.run(() -> m_magazine.setFeedVelocity(-0.3)).alongWith(Commands.run(() -> m_drive.lockWheels()))); //just runs feed/mag

        // m_xboxController0.rightBumper().onTrue(new IntakeRetract(m_intake)); // works 50/50 - zane   
        //m_xboxController0.rightTrigger().onTrue(new SendHoodToHome(m_shooter));
        //m_xboxController0.povUp().onTrue(Commands.run(() -> BumpShooterVector.bumpLaunchAngleUp())); //Angle up
        //m_xboxController0.povDown().onTrue(Commands.run(() -> BumpShooterVector.bumpLaunchAngleDown())); // Angle down
        //m_xboxController0.povLeft().onTrue(Commands.run(() -> BumpShooterVector.bumpLaunchSpeedUp())); //speed up
        //m_xboxController0.povRight().onTrue(Commands.run(() -> BumpShooterVector.bumpLaunchSpeedDown())); //speed down
        //m_xboxController0.a().whileTrue(new RevShooter( 280, 72, m_shooter, m_xboxController0));//rev hub //TODO addvibration last priority split rev and fire 
        //m_xboxController0.x().whileTrue(new RevShooter(377, 65, m_shooter, m_xboxController0));//rev bump
        //m_xboxController0.y().whileTrue(new RevShooter( 335, 65, m_shooter, m_xboxController0));//rev tower
        //m_xboxController0.b().whileTrue(new FireShooter( 1000, ShooterConstants.hoodLowerLimit_deg, m_shooter, m_drive, m_magazine));//rev neutral */
        //   m_xboxController0.back().whileTrue(Commands.startEnd(() -> m_intake.intakeRun(0.8), () -> m_intake.intakeRollerStop(), m_intake));//TODO: test
        //   m_xboxController0.start().whileTrue(Commands.startEnd(() -> m_magazine.setFeedVelocity(-0.3), () -> m_magazine.cancel(), m_magazine));//TODO: 
        //   //m_xboxController0.mode().whileTrue(commands.startEnd(() -> m_shooter.unjam(), () -> m_shooter.cancel(), m_shooter)); mode dosent exsist 
        //   m_xboxController0.leftStick().onTrue(Commands.run(() -> BumpShooterVector.reset())); // reset angle/speed
        //   m_xboxController0.rightStick().onTrue(new StopRobot(m_intake, m_magazine, m_climbe, m_drive, m_shooter));
    }

    private void configureJoystick() {
        m_joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
        m_joystick.button(2).onTrue(Commands.print("Joystick Button 2"));
    }

    private void configureFlightController() {
        new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
                .onTrue(new ResetGyro(m_gyro));
        new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
                .onTrue(Commands.runOnce(() -> m_stickCommand.lockWheels()));
        new Trigger(() -> m_flightController0.getIButton())
                .onTrue(Commands.runOnce(() -> m_drive.resetPose(new Pose2d())));
    }

    private void configurePS4Controller() {
        // Hold 'Cross' to test flywheels (Already there)
        m_ps4Controller.cross().whileTrue(
                Commands.startEnd(
                        () -> m_shooter.setShooterVelocity(500.0),
                        () -> m_shooter.cancel(),
                        m_shooter
                )
        );

        // --- NEW: PS4 DRIVE TEST ---
        // This maps the PS4 sticks to the robot's movement.
        // Left Stick: Move (X/Y) | Right Stick: Rotate (Omega)
        m_drive.setDefaultCommand(
                Commands.run(
                        () -> {
                            // We negate the Y because forward on a joystick is -1.0
                            double xSpeed = -modifyAxis(m_ps4Controller.getLeftY()) * Constants.DriveConstants.maxSpeedMetersPerSec;
                            double ySpeed = -modifyAxis(m_ps4Controller.getLeftX()) * Constants.DriveConstants.maxSpeedMetersPerSec;
                            double rotSpeed = -modifyAxis(m_ps4Controller.getRightX()) * m_drive.getMaxAngularSpeedRadPerSec();

                            m_drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            xSpeed, ySpeed, rotSpeed, m_gyro.getRotationAs2d()
                                    )
                            );
                        },
                        m_drive
                )
        );
    }

    /**
     * Simple deadband helper to stop the robot from drifting due to worn-out
     * PS4 controller sticks.
     */
    private double modifyAxis(double value) {
        if (Math.abs(value) < 0.1) {
            return 0.0; // 10% deadband

                }return value;
    }

    public Command getAutonomousCommand() {
        // return new RightMidwayGather(m_drive, m_wheels, m_gyro, m_shooter, m_magazine, m_intake);
        // if (m_autonSelector == null) {
        //   return Commands.none();
        // }

        // Command selectedCommand = m_autonSelector.getSelected();
        // if (selectedCommand != null) {
        //   System.out.println("Selected auton: " + selectedCommand);
        // } else {
        //   System.out.println("Selected auton: <null>");
        // }
        return autoChooser.get();
    }
}
