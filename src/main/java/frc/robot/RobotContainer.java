package frc.robot;

import frc.lib.SpektrumFlightController;

import frc.robot.subsystems.ChooserHandeling;
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.SwerveModule;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;
import frc.robot.subsystems.Vision.AprilTags.RebuiltAprilTags;
import frc.robot.commands.MovingRotateToTag;
import frc.robot.commands.AutonomousCommands.RotateToAngle;
import frc.robot.commands.AutonomousCommands.RotateToTag;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;

public class RobotContainer {

  private final Drivetrain m_Drive;
  private final IGyro m_Gyro = new NavxGyro();

  private final SpektrumFlightController m_flightController0 = new SpektrumFlightController(0);
  private final CommandXboxController m_xboxController0 = new CommandXboxController(1);
  private final CommandJoystick m_joystick = new CommandJoystick(2);
  private final StickCommand m_stickCommand;

  public final SwerveModule[] m_wheels;
  private final VisionServer m_visionServer = new VisionServer();
  // private final Shooter m_shooter = new Shooter(); TODO uncomment when CAN Ids
  // are assigned

  private final AprilTagFilter m_aprilTagFilter = new AprilTagFilter(RebuiltAprilTags.m_tags);
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
            m_selectedBot.getfrontLeftLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.backLeftSteerMotorID,
            Constants.CANIdConstants.backLeftDriveMotorID,
            Constants.CANIdConstants.backLeftEncoderid,
            m_selectedBot.getbackLeftOffset(),
            m_selectedBot.getbackLeftLocationX(),
            m_selectedBot.getbackLeftLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.frontRightSteerMotorID,
            Constants.CANIdConstants.frontRightDriveMotorID,
            Constants.CANIdConstants.frontRightEncoderid,
            m_selectedBot.getfrontRightOffset(),
            m_selectedBot.getfrontRightLocationX(),
            m_selectedBot.getfrontRightLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.backRightSteerMotorID,
            Constants.CANIdConstants.backRightDriveMotorID,
            Constants.CANIdConstants.backRightEncoderid,
            m_selectedBot.getbackRightOffset(),
            m_selectedBot.getbackRightLocationX(),
            m_selectedBot.getbackRightLocationY())
    };

    m_Drive = new Drivetrain(m_wheels, m_Gyro);

    configureFlightControllerParameters();

    m_stickCommand = new StickCommand(
        m_Drive,
        m_Gyro,
        m_selectedBot,
        () -> m_flightController0.getDriveX(),
        () -> m_flightController0.getDriveY(),
        () -> m_flightController0.getRotate(),
        () -> m_flightController0.getThrottle(),
        () -> m_flightController0.getSelectPressButton());

    configureBindings();

    // Modern automatic default command
    m_Drive.setDefaultCommand(m_stickCommand);

    m_autonSelector = new ChooserHandeling("Auton Selection", 
      Map.ofEntries(
        Map.entry("00 Do Nothing", () -> Commands.none())
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
    xboxController0.rightBumper().onTrue(new RotateToAngle(m_Drive, 90, m_Gyro));
    xboxController0.leftBumper().onTrue(new StraightLineDrive(m_Drive, 180, m_wheels));
    Trigger aButton = xboxController0.a();
    Trigger bButton = xboxController0.b();
    Trigger xButton = xboxController0.x();
    Trigger yButton = xboxController0.y();
    
    aButton.whileTrue(m_wheels[0].sysIdQuasistaticForward());
    bButton.whileTrue(m_wheels[0].sysIdQuasistaticReverse());
    xButton.whileTrue(m_wheels[0].sysIdDynamicForward());
    yButton.whileTrue(m_wheels[0].sysIdDynamicReverse());
    
    aButton.whileTrue(m_wheels[1].sysIdQuasistaticForward());
    bButton.whileTrue(m_wheels[1].sysIdQuasistaticReverse());
    xButton.whileTrue(m_wheels[1].sysIdDynamicForward());
    yButton.whileTrue(m_wheels[1].sysIdDynamicReverse());

    aButton.whileTrue(m_wheels[2].sysIdQuasistaticForward());
    bButton.whileTrue(m_wheels[2].sysIdQuasistaticReverse());
    xButton.whileTrue(m_wheels[2].sysIdDynamicForward());
    yButton.whileTrue(m_wheels[2].sysIdDynamicReverse());

    aButton.whileTrue(m_wheels[3].sysIdQuasistaticForward());
    bButton.whileTrue(m_wheels[3].sysIdQuasistaticReverse());
    xButton.whileTrue(m_wheels[3].sysIdDynamicForward());
    yButton.whileTrue(m_wheels[3].sysIdDynamicReverse());
  }

  private void configureJoystick() {
    m_joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
    m_joystick.button(2).onTrue(Commands.print("Joystick Button 2"));
  }

  private void configureFlightController() {
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kReset.value))
        .onTrue(new ResetGyro(m_Gyro));
    new Trigger(() -> m_flightController0.getRawButton(SpektrumFlightController.Button.kSelectPress.value))
        .onTrue(new LockWheels(m_Drive));
    new Trigger(() -> m_flightController0.getIButton())
        .whileTrue(new MovingRotateToTag(m_visionServer, m_stickCommand));
  }

  public Command getAutonomousCommand() {

    if (m_autonSelector == null){
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
