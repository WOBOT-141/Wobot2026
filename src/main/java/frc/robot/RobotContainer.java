package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.SpektrumFlightController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.ChooserHandeling;

import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.TestCommands.DriveInDirection;

/**
 * RobotContainer defines:
 * - Subsystems
 * - Default commands
 * - Controller bindings
 *
 * This is the heart of the robot.
 */
public class RobotContainer {

  private final Drivetrain m_Drive;
  private final IGyro m_Gyro = new NavxGyro();

  // Controllers (NOT static — prevents hidden dependencies)
  private final SpektrumFlightController flightController0 =
      new SpektrumFlightController(0);

  private final CommandXboxController xboxController0 =
      new CommandXboxController(1);

  private final CommandJoystick joystick =
      new CommandJoystick(2);

  public final SwerveModule[] m_wheels;

  private final IBotSelector _selectedBot = new MainBot();
  // private final IBotSelector _selectedBot = new SummerBot();

  ChooserHandeling autonSelector;

  /**
   * Construct subsystems and configure bindings.
   */
  public RobotContainer() {

    m_wheels = new SwerveModule[] {
        // assuming that the roborio is the front and the battery is the back
        new SwerveModule(
            Constants.CANIdConstants.frontLeftSteerMotorID,
            Constants.CANIdConstants.frontLeftDriveMotorID,
            Constants.CANIdConstants.frontLeftEncoderid,
            _selectedBot.getfrontLeftOffset(),
            _selectedBot.getfrontLeftLocationX(),
            _selectedBot.getfrontLeftLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.backLeftSteerMotorID,
            Constants.CANIdConstants.backLeftDriveMotorID,
            Constants.CANIdConstants.backLeftEncoderid,
            _selectedBot.getbackLeftOffset(),
            _selectedBot.getbackLeftLocationX(),
            _selectedBot.getbackLeftLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.frontRightSteerMotorID,
            Constants.CANIdConstants.frontRightDriveMotorID,
            Constants.CANIdConstants.frontRightEncoderid,
            _selectedBot.getfrontRightOffset(),
            _selectedBot.getfrontRightLocationX(),
            _selectedBot.getfrontRightLocationY()),

        new SwerveModule(
            Constants.CANIdConstants.backRightSteerMotorID,
            Constants.CANIdConstants.backRightDriveMotorID,
            Constants.CANIdConstants.backRightEncoderid,
            _selectedBot.getbackRightOffset(),
            _selectedBot.getbackRightLocationX(),
            _selectedBot.getbackRightLocationY())
    };

    m_Drive = new Drivetrain(m_wheels, m_Gyro);

    configureBindings();

    // Automatic default drive command (modern approach)
    m_Drive.setDefaultCommand(
        new StickCommand(
            m_Drive,
            m_Gyro,
            _selectedBot,
            () -> flightController0.getDriveX(),
            () -> flightController0.getDriveY(),
            () -> flightController0.getRotate(),
            () -> flightController0.getThrottle(),
            () -> flightController0.getSelectPressButton()
        )
    );
  }

  /**
   * Re-zero swerve modules (called from Robot.disabledPeriodic()).
   */
  public void wheelSetup() {

    m_wheels[0].setEncoderOffset(_selectedBot.getfrontLeftOffset());
    m_wheels[0].set_xPos(_selectedBot.getfrontLeftLocationX());
    m_wheels[0].set_yPos(_selectedBot.getfrontLeftLocationY());

    m_wheels[1].setEncoderOffset(_selectedBot.getbackLeftOffset());
    m_wheels[1].set_xPos(_selectedBot.getbackLeftLocationX());
    m_wheels[1].set_yPos(_selectedBot.getbackLeftLocationY());

    m_wheels[2].setEncoderOffset(_selectedBot.getfrontRightOffset());
    m_wheels[2].set_xPos(_selectedBot.getfrontRightLocationX());
    m_wheels[2].set_yPos(_selectedBot.getfrontRightLocationY());

    m_wheels[3].setEncoderOffset(_selectedBot.getbackRightOffset());
    m_wheels[3].set_xPos(_selectedBot.getbackRightLocationX());
    m_wheels[3].set_yPos(_selectedBot.getbackRightLocationY());

    for (int i = 0; i < 4; i++) {
      m_wheels[i].settoZero();
    }
  }

  /**
   * Configure controller bindings.
   */
  private void configureBindings() {
    configureXboxController();
    configureJoystick();
    configureFlightController();
  }

  private void configureXboxController() {

    xboxController0.a()
        .onTrue(new ResetGyro(m_Gyro));

    xboxController0.rightBumper()
        .onTrue(new LockWheels(m_Drive));

    xboxController0.leftBumper()
        .whileTrue(new DriveInDirection(m_Drive, 1, 0.02));

    xboxController0.povUp()
        .onTrue(Commands.print("POV Up Pressed"));
  }

  private void configureJoystick() {

    joystick.trigger()
        .onTrue(Commands.print("Joystick Trigger"));

    joystick.button(2)
        .onTrue(Commands.print("Joystick Button 2"));
  }

  /**
   * SpektrumFlightController buttons wrapped manually.
   */
  private void configureFlightController() {

    SmartDashboard.putBoolean(
        "Flight Controller Connected",
        flightController0.isConnected());

    new Trigger(
        () -> flightController0.getRawButton(
            SpektrumFlightController.Button.kReset.value))
        .onTrue(new ResetGyro(m_Gyro));
  }

  /**
   * Returns selected autonomous command.
   */
  public Command getAutonomousCommand() {

    Command selectedCommand = autonSelector.getSelected();

    if (selectedCommand != null) {
      System.out.println("Selected auton: " + selectedCommand);
    } else {
      System.out.println("Selected auton: <null>");
    }

    return selectedCommand;
  }
}

