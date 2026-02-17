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
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;

public class RobotContainer {

  private final Drivetrain m_Drive;
  private final IGyro m_Gyro = new NavxGyro();

  private final SpektrumFlightController flightController0 =
      new SpektrumFlightController(0);

  private final CommandXboxController xboxController0 =
      new CommandXboxController(1);

  private final CommandJoystick joystick =
      new CommandJoystick(2);

  public final SwerveModule[] m_wheels;

  private final IBotSelector _selectedBot = new MainBot();

  ChooserHandeling autonSelector;

  public RobotContainer() {

    m_wheels = new SwerveModule[] {

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

    // 🔥 Restore deadband + stick parameter setup
    configureFlightControllerParameters();

    configureBindings();

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
   * Configure Spektrum flight controller scaling and deadbands.
   * This replaces the old deadbandSetup() method.
   */
  private void configureFlightControllerParameters() {

    IStickParameters p = new StickBParameters();

    flightController0.setRotateFactor(p.get_RotateFactor());
    flightController0.setDriveXFactor(p.get_DriveXFactor());
    flightController0.setDriveYFactor(p.get_DriveYFactor());
    flightController0.setThrottleFactor(p.get_ThrottleFactor());

    flightController0.setRotateOffset(p.get_RotateOffset());
    flightController0.setDriveXOffset(p.get_DriveXOffset());
    flightController0.setDriveYOffset(p.get_DriveYOffset());
    flightController0.setThrottleOffset(p.get_ThrottleOffset());

    flightController0.setKnobOffset(p.get_knobOffset());
    flightController0.setxDeadBand(p.get_deadbandX());
    flightController0.setyDeadBand(p.get_deadbandY());
    flightController0.setRotationDeadBand(p.get_deadbandRot());
  }

  private void configureBindings() {
    configureXboxController();
    configureJoystick();
    configureFlightController();
  }

  private void configureXboxController() {

    xboxController0.a().onTrue(new ResetGyro(m_Gyro));
    xboxController0.rightBumper().onTrue(new LockWheels(m_Drive));
    xboxController0.leftBumper().whileTrue(new DriveInDirection(m_Drive, 1, 0.02));
    xboxController0.povUp().onTrue(Commands.print("POV Up Pressed"));
  }

  private void configureJoystick() {

    joystick.trigger().onTrue(Commands.print("Joystick Trigger"));
    joystick.button(2).onTrue(Commands.print("Joystick Button 2"));
  }

  private void configureFlightController() {

    SmartDashboard.putBoolean(
        "Flight Controller Connected",
        flightController0.isConnected());

    new Trigger(
        () -> flightController0.getRawButton(
            SpektrumFlightController.Button.kReset.value))
        .onTrue(new ResetGyro(m_Gyro));
  }

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

