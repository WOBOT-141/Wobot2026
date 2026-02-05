// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.SpektrumFlightController;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.ChooserHandeling;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;
>>>>>>> Stashed changes

public class RobotContainer {
<<<<<<< Updated upstream
  private PneumaticHub pneumaticHub = new PneumaticHub(27);
  private final Drivetrain m_Drive;

  public static SpektrumFlightController flightController0;
  public static XboxController xboxController0;
  public static Joystick joystick;
  private final IGyro m_Gyro = new NavxGyro();
  public final SwerveModule[] m_wheels;

  private final IBotSelector _selectedBot = new SummerBot();
  // private final IBotSelector _selectedBot = new SummerBot();
=======

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();
>>>>>>> Stashed changes

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

<<<<<<< Updated upstream
    m_wheels = new SwerveModule[] {
        // assuming that the roborio is the front and the battery is the back
        new SwerveModule(Constants.SteerConstants.frontLeftSteerMotorID,
            Constants.SteerConstants.frontLeftDriveMotorID,
            Constants.SteerConstants.frontLeftEncoderid,
            _selectedBot.getfrontLeftOffset(),
            Constants.SteerConstants.frontLeftMotorInvert,
            Constants.SteerConstants.frontLeftEncoderInvert,
            _selectedBot.getfrontLeftLocationX(),
            _selectedBot.getfrontLeftLocationY()), // front left 0.441650 rotations

        new SwerveModule(Constants.SteerConstants.backLeftSteerMotorID,
            Constants.SteerConstants.backLeftDriveMotorID,
            Constants.SteerConstants.backLeftEncoderid,
            _selectedBot.getbackLeftOffset(),
            Constants.SteerConstants.backLeftMotorInvert,
            Constants.SteerConstants.backLeftEncoderInvert,
            _selectedBot.getbackLeftLocationX(),
            _selectedBot.getbackLeftLocationY()), // back left 0.176270 rotations

        new SwerveModule(Constants.SteerConstants.frontRightSteerMotorID,
            Constants.SteerConstants.frontRightDriveMotorID,
            Constants.SteerConstants.frontRightEncoderid,
            _selectedBot.getfrontRightOffset(),
            Constants.SteerConstants.frontRightMotorInvert,
            Constants.SteerConstants.frontRightEncoderInvert,
            _selectedBot.getfrontRightLocationX(),
            _selectedBot.getfrontRightLocationY()), // front right 0.585205 rotations

        new SwerveModule(Constants.SteerConstants.backRightSteerMotorID,
            Constants.SteerConstants.backRightDriveMotorID,
            Constants.SteerConstants.backRightEncoderid,
            _selectedBot.getbackRightOffset(),
            Constants.SteerConstants.backRightMotorInvert,
            Constants.SteerConstants.backRightEncoderInvert,
            _selectedBot.getbackRightLocationX(),
            _selectedBot.getbackRightLocationY())// back right 0.506836 rotations
    };
    m_Drive = new Drivetrain(m_wheels, m_Gyro);
    
    pneumaticHub.enableCompressorDigital();

    flightController0 = new SpektrumFlightController(0);
    xboxController0 = new XboxController(1);
    joystick = new Joystick(2);

    configureBindings();
  }

  public void deadbandSetup() {
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

  public boolean isPlugged() {
    return flightController0.isConnected();
  }

  public void stickCommandInit() {
    deadbandSetup();

    m_StickCommand = new StickCommand(m_Drive, m_Gyro, flightController0, _selectedBot);
    m_Drive.setDefaultCommand(m_StickCommand);
    configureRotateBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * { @link Trigger#Trigger(java.util.function.Bo
   * // leanSupplier)} constructor with
   * an arbitrary
   * 
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureRotateBindings() {
    // Trigger A = new JoystickButton(flightController0,
    // SpektrumFlightController.Button.kA.value);
    // A.whileTrue(new ArcCommand(m_StickCommand));
    // SmartDashboard.putBoolean("isRotating around center", A.getAsBoolean());
  }

  private void configureBindings() {
    configureXboxController();
    configureJoystick();
    configureFlightController();
  }

  private void configureXboxController() {
    Trigger leftBumper = new JoystickButton(xboxController0, XboxController.Button.kLeftBumper.value);
    Trigger rightBumper = new JoystickButton(xboxController0, XboxController.Button.kRightBumper.value);
    Trigger leftTrigger = new Trigger(() -> {return xboxController0.getLeftTriggerAxis() >= .5;});
    Trigger rightTrigger = new Trigger(() -> {return xboxController0.getRightTriggerAxis() >= .5; });
    Trigger aButton = new JoystickButton(xboxController0, XboxController.Button.kA.value);
    Trigger bButton = new JoystickButton(xboxController0, XboxController.Button.kB.value);
    Trigger xButton = new JoystickButton(xboxController0, XboxController.Button.kX.value);
    Trigger yButton = new JoystickButton(xboxController0, XboxController.Button.kY.value);
    Trigger dPadUp = new POVButton(xboxController0, 0);
    Trigger dPadRight = new POVButton(xboxController0, 90);
    Trigger dPadDown = new POVButton(xboxController0, 180);
    Trigger dPadLeft = new POVButton(xboxController0, 270);
  }

  private void configureJoystick() {
    Trigger trigger = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);
    Trigger sideButton = new JoystickButton(joystick, 2);
    Trigger seven = new JoystickButton(joystick, 7);
    Trigger eight = new JoystickButton(joystick, 8);
    Trigger twelve = new JoystickButton(joystick, 12);
  }

  private void configureFlightController() {
    Trigger lockWheels = new JoystickButton(flightController0, SpektrumFlightController.Button.kSelectPress.value);
    Trigger reset = new JoystickButton(flightController0, SpektrumFlightController.Button.kReset.value);
    Trigger cancel = new JoystickButton(flightController0, SpektrumFlightController.Button.kCancel.value);

    reset.onTrue(new ResetGyro(m_Gyro));
    lockWheels.onTrue(new LockWheels(m_Drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selectedCommand = autonSelector.getSelected();

    // Command selectedCommand = new ObliterateCenterTwoA1(m_Drive, m_Gyro,
    // m_Elevator, m_coralManipulator, serveVis);

    if (selectedCommand != null) {
      System.out.println("Selected auton: " + selectedCommand.toString());
    } else {
      System.out.println("Selected auton: <null>");
=======
        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
=======
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.StickBParameters; // ← add this import
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.Wheel;

public class RobotContainer {

    private final Drive m_drive;
    private final IGyro m_gyro = new NavxGyro();
   
    public static SpektrumFlightController flightController;
    public static XboxController xboxController;
    public static Joystick joystick;

    private final IBotSelector selectedBot = new MainBot();

    private StickCommand m_stickCommand;

    public RobotContainer() {
        Wheel[] wheels = new Wheel[] {
                new Wheel(
                        Constants.SteerConstants.frontLeftSteerMotorID,
                        Constants.SteerConstants.frontLeftDriveMotorID,
                        Constants.SteerConstants.frontLeftEncoderid,
                        selectedBot.getfrontLeftOffset(),
                        Constants.SteerConstants.frontLeftMotorInvert,
                        selectedBot.getfrontLeftLocationX(),
                        selectedBot.getfrontLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.backLeftSteerMotorID,
                        Constants.SteerConstants.backLeftDriveMotorID,
                        Constants.SteerConstants.backLeftEncoderid,
                        selectedBot.getbackLeftOffset(),
                        Constants.SteerConstants.backLeftMotorInvert,
                        selectedBot.getbackLeftLocationX(),
                        selectedBot.getbackLeftLocationY()),
                new Wheel(
                        Constants.SteerConstants.frontRightSteerMotorID,
                        Constants.SteerConstants.frontRightDriveMotorID,
                        Constants.SteerConstants.frontRightEncoderid,
                        selectedBot.getfrontRightOffset(),
                        Constants.SteerConstants.frontRightMotorInvert,
                        selectedBot.getfrontRightLocationX(),
                        selectedBot.getfrontRightLocationY()),
                new Wheel(
                        Constants.SteerConstants.backRightSteerMotorID,
                        Constants.SteerConstants.backRightDriveMotorID,
                        Constants.SteerConstants.backRightEncoderid,
                        selectedBot.getbackRightOffset(),
                        Constants.SteerConstants.backRightMotorInvert,
                        selectedBot.getbackRightLocationX(),
                        selectedBot.getbackRightLocationY())
        };

        m_drive = new Drive(wheels); // ← only Wheel[] parameter

        flightController = new SpektrumFlightController(0);
        xboxController = new XboxController(1);
        joystick = new Joystick(2);

        configureDefaultCommand();
        configureBindings();
>>>>>>> Stashed changes
    }

    private void configureDefaultCommand() {
        var params = new StickBParameters();

        flightController.setRotateFactor(params.get_RotateFactor());
        flightController.setDriveXFactor(params.get_DriveXFactor());
        flightController.setDriveYFactor(params.get_DriveYFactor());
        flightController.setThrottleFactor(params.get_ThrottleFactor());
        flightController.setRotateOffset(params.get_RotateOffset());
        flightController.setDriveXOffset(params.get_DriveXOffset());
        flightController.setDriveYOffset(params.get_DriveYOffset());
        flightController.setThrottleOffset(params.get_ThrottleOffset());
        flightController.setxDeadBand(params.get_deadbandX());
        flightController.setyDeadBand(params.get_deadbandY());
        flightController.setRotationDeadBand(params.get_deadbandRot());

        m_stickCommand = new StickCommand(m_drive, m_gyro, flightController, selectedBot);
        m_drive.setDefaultCommand(m_stickCommand);
    }

    private void configureBindings() {
        // Flight controller
        new JoystickButton(flightController, SpektrumFlightController.Button.kSelectPress.value)
                .onTrue(new LockWheels(m_drive));

        new JoystickButton(flightController, SpektrumFlightController.Button.kReset.value)
                .onTrue(new ResetGyro(m_gyro));

        new JoystickButton(RobotContainer.flightController, SpektrumFlightController.Button.kReset.value)
                .onTrue(Commands.runOnce(() -> {
                    m_gyro.zeroYaw();
                    System.out.println("Gyro yaw zeroed!"); // this is for troubleshooting and can be removed or commented out
                }));

        // Xbox example bindings (replace with real commands)
        CommandXboxController xbox = new CommandXboxController(1);

        xbox.leftBumper().whileTrue(Commands.print("Left bumper held"));
        xbox.rightBumper().whileTrue(Commands.print("Right bumper held"));
        xbox.a().onTrue(Commands.print("A pressed"));
        // ... add your actual commands here
    }

    public Command getAutonomousCommand() {
        return Commands.none(); // replace with real auton
    }
}