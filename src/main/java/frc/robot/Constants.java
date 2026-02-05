// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean  constants. This class should not be used for any other 
 * purpose. All constants should be declared globally (i.e. public static). 
 * Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public static class RotateCommandConstants {
    public static final double P = 0.0035;
    public static final double I = 0.00000;
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance = 3.0; //2.0
  }

  public static class RotateVisionConstants {
    public static final double P = 0.004;
    public static final double I = 0.002;//.00001
    public static final double D = 0.00000;
    public static final double rotateTargetTolerance = 2; //2.0
  }

  public static class DriveCommandConstants {

    public static final double P = 0.024;//.0024
    public static final double I = 0.0008;
    public static final double D = 0;

    public static final double P2 = .04;
    public static final double I2 = .00012;
  }

  public static class DriveConstants {
    // wheel diameter multiplied by pi = circumference
<<<<<<< Updated upstream
    public static final double wheelCircumference = (Math.PI * 4);
    public static final double driveGearRatio = 6.12;
    // the number of motor rotations to get one wheel rotation (3.6) divided by the
    // wheel circumference
    public static final double rotationsPerInch = driveGearRatio / wheelCircumference; // 3.6 might be the gear ratio
    public static final double metersToRotationsPerMin = driveGearRatio / Units.inchesToMeters(wheelCircumference) * 60; // RPS
=======
    public static final double wheelCircumference_in = (Math.PI * 4);
    public static final double driveGearRatio = 5.14;
    
    // Number of inches the first stage moves for every rotation of the motors
    public static final double inchesPerRotation = driveGearRatio * wheelCircumference_in;
    
    // Number of inches carriage moves per rotation of the motors (twice what the first stage moves)
    public static final double carriageInchesPerRotation = inchesPerRotation * 2;

    public static final double rotationsPerInch = driveGearRatio / wheelCircumference_in; // 3.6 might be the gear ratio
    public static final double metersToRotationsPerMin = driveGearRatio / Units.inchesToMeters(wheelCircumference_in)
        * 60; // RPS
    
    public static final int driveCurrentLim_amp = 80;
    public static final double driveTargetTolerance = 2.0;
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
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
    public static final double drivekP = 0.0016;// .00009
    public static final double drivekI = 0.000068;// 0
    public static final double drivekD = 0;
    public static final double drivekIz = 0;
    public static final double drivekFF = 0.00016500000492669642;
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
    // top speed of bot in feet per second converted into meters per second
    public static final double topSpeed = 25;// 20.36

    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    public static final int driveCurrentLim = 80;

    public static final double driveTargetTolerance = 2.0;
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
=======
    public static final double topSpeed_mps = 25;// 20.36 // top speed of bot in feet per second converted into meters per second
    
    public static final double autoRotatekP = 0.4; // 0.72
    public static final double autoRotatekI = 0.02;
    public static final double autoRotatekD = 0;
    
>>>>>>> Stashed changes
    public static final double visionDrivekP = 0.0032;
    public static final double visionDrivekI = 0.00028;
    public static final double visionDrivekD = 0.00015;
  }

  public static class SteerConstants {
    public static final double encoderPositionConversionFactor = 28.13;
    public static final double kF = 0;
    public static final double kP = .05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int steerContCurrentLim = 4;
    public static final int steerPeakCurrentLim = 6;
    public static final boolean frontRightEncoderInvert = true;
    public static final boolean backRightEncoderInvert = false;
    public static final boolean frontLeftEncoderInvert = false;
    public static final boolean backLeftEncoderInvert = true;
    public static final boolean frontLeftMotorInvert = false; // true
    public static final boolean backLeftMotorInvert = false; // true
    public static final boolean frontRightMotorInvert = true; // false
    public static final boolean backRightMotorInvert = true; // false
    public static final int frontLeftEncoderid = 9; // 9
    public static final int backLeftEncoderid = 10; // 10
    public static final int backRightEncoderid = 11; // 11
    public static final int frontRightEncoderid = 12; // 12
    public static final int frontLeftSteerMotorID = 1;
    public static final int frontLeftDriveMotorID = 2;
    public static final int backLeftSteerMotorID = 3;
    public static final int backLeftDriveMotorID = 4;
    public static final int frontRightSteerMotorID = 5;
    public static final int frontRightDriveMotorID = 6;
    public static final int backRightSteerMotorID = 7;
    public static final int backRightDriveMotorID = 8;
  }

  public static class ManipulatorConstants {
    public static final int rightIntakeMotor = 18;
    public static final int leftIntakeMotor = 19;
    public static final int algaeIntakeMotor = 40;
    public static final int algaeSolenod = 15;
    public static final int climbSolenoid1 = 4;
    public static final int climbSolenoid2 = 3;
    public static final int clawSolenoid = 1;
    public static final int climbMotorLeftId = 21;
    public static final int climbMotorRightId = 22;
  }

  public static class WinchClimberConstants {
    public static final int leftRollerID = 31;
    public static final int rightRollerID = 32;
    public static final int armMotorID = 33;
    public static final int winchMotorID = 34;
  }

  public static class LidarConstants {
    public static final int leftLidar = 40;
    public static final int rightLidar = 41;
    public static final int forwardLidarLeft = 42;
    public static final int forwardLidarRight = 43;
  }

  public static class ElevatorConstants {
    public static final int elevatorTipForward = 6;
    public static final int elevatorTipBack = 0;
    public static final int lowerLeftMotorID = 14;
    public static final int upperLeftMotorID = 15;
    public static final int lowerRightMotorID = 16;
    public static final int upperRightMotorID = 17;

    public static final double elevatorMotorP = .025;
    public static final double elevatorMotorI = 0.0000; //
    public static final double elevatorMotorD = 0.0000; //
    public static final double gravityFeedForward = .75;

    public static final double elevatorMotorPM = .180; //.150

    public static final double elevatorMotorP2 = .0025; //
    public static final double elevatorMotorI2 = 0.00000035; // .00001 //.000005 //0.0000035
    public static final double elevatorMotorD2 = 0.0; // .5

    public static final double maxElevatorHeight = 78.5; // TODO make actual value
    public static final double minElevatorHeight = -.5; // TODO make actual value
    public static final double minSafeTipBackHeight = 5; // TODO make actual value AND maybe give better name?
    public static final double homeHeight = 0.18; // TODO make actual value
    // Ratio of the gearbox for the elevator, 7 rotations of the motor = one
    // rotation of the gearbox hence 1/7
    public static final double driveGearRatio = 1.0 / 7.0;
    // Circumference of the wheel on the output end of the gearbox, 1 roation of the
    // gearbox = 7.086678 inches travelled
    public static final double wheelCircumference = 2.2557 * Math.PI;
    // Number of inches the first stage moves for every rotation of the motors
    public static final double inchesPerRotation = driveGearRatio * wheelCircumference;
    // Number of inches carriage moves per rotation of the motors (twice what the
    // first stage moves)
    public static final double carriageInchesPerRotation = inchesPerRotation * 2;

    public static class ControllerOneConstants {
      /**
       * Factors used in the spektrum flight controller to multiply the raw stick
       * outputs in order to scale them up or down.
       * These can be found by dividing 1 by the raw stick output of that direction
       */
      public static final double RotateFactor = 1.35;
      public static final double DriveXFactor = 1.26;
      public static final double DriveYFactor = 1.23;
      public static final double ThrottleFactor = 1.55;
      /**
       * Offsets used to add or subtract from a raw stick value in the spektrum flight
       * controller to make their resting postion 0
       * These should be equal to the raw stick output at rest.
       */
      public static final double RotateOffset = -0.056375;
      public static final double DriveXOffset = 0.039816 + 0.029202; // .075813
      public static final double DriveYOffset = -0.024228;
      public static final double ThrottleOffset = 0;

      // Deadbands for sticks
      public static final double deadbandRot = 0.025;
      public static final double deadbandX = 0.025;
      public static final double deadbandY = 0.025;
    }
  }
}
