// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.SpektrumFlightController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.NavxGyro;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.ChooserHandeling;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RotateToTag;
import frc.robot.commands.DriveCommands.LockWheels;
import frc.robot.commands.DriveCommands.ResetGyro;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.StickParameters.IStickParameters;
import frc.robot.commands.StickParameters.StickBParameters;
import frc.robot.commands.TestCommands.DriveInDirection;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain m_Drive;

  public static SpektrumFlightController flightController0;
  public static XboxController xboxController0;
  public static Joystick joystick;
  private final IGyro m_Gyro = new NavxGyro();
  public final SwerveModule[] m_wheels;
  private final VisionServer visionServer;

  private final IBotSelector _selectedBot = new MainBot();
  // private final IBotSelector _selectedBot = new SummerBot();

  StickCommand m_StickCommand;

  ChooserHandeling autonSelector;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    m_wheels = new SwerveModule[] {
        // assuming that the roborio is the front and the battery is the back
        new SwerveModule(Constants.SteerConstants.frontLeftSteerMotorID,
            Constants.SteerConstants.frontLeftDriveMotorID,
            Constants.SteerConstants.frontLeftEncoderid,
            _selectedBot.getfrontLeftOffset(),
            _selectedBot.getfrontLeftLocationX(),
            _selectedBot.getfrontLeftLocationY()), // front left 0.441650 rotations

        new SwerveModule(Constants.SteerConstants.backLeftSteerMotorID,
            Constants.SteerConstants.backLeftDriveMotorID,
            Constants.SteerConstants.backLeftEncoderid,
            _selectedBot.getbackLeftOffset(),
            _selectedBot.getbackLeftLocationX(),
            _selectedBot.getbackLeftLocationY()), // back left 0.176270 rotations

        new SwerveModule(Constants.SteerConstants.frontRightSteerMotorID,
            Constants.SteerConstants.frontRightDriveMotorID,
            Constants.SteerConstants.frontRightEncoderid,
            _selectedBot.getfrontRightOffset(),
            _selectedBot.getfrontRightLocationX(),
            _selectedBot.getfrontRightLocationY()), // front right 0.585205 rotations

        new SwerveModule(Constants.SteerConstants.backRightSteerMotorID,
            Constants.SteerConstants.backRightDriveMotorID,
            Constants.SteerConstants.backRightEncoderid,
            _selectedBot.getbackRightOffset(),
            _selectedBot.getbackRightLocationX(),
            _selectedBot.getbackRightLocationY())// back right 0.506836 rotations
    };
    m_Drive = new Drivetrain(m_wheels, m_Gyro);
    
    flightController0 = new SpektrumFlightController(0);
    xboxController0 = new XboxController(1);
    joystick = new Joystick(2);
    visionServer = new VisionServer();

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
    bButton.whileTrue(new RotateToTag(visionServer, m_Drive));
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
    Trigger shoulderButton = new JoystickButton(flightController0, SpektrumFlightController.Button.kI.value);

    reset.onTrue(new ResetGyro(m_Gyro));
    lockWheels.onTrue(new LockWheels(m_Drive));
    shoulderButton.whileTrue(new DriveInDirection(m_Drive, 1, 0.02));
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
    }

    return selectedCommand;

    // return new RotateCommandField(m_Drive, m_Gyro, 180);

    // return new UniversalDriveCommandPicture(m_Drive, m_Gyro, .5, serveVis);

    // return new VisionL4Experimental(m_Drive, serveVis, m_Gyro, m_Elevator,
    // m_coralManipulator, ReefSide.LEFT);
  }
}
