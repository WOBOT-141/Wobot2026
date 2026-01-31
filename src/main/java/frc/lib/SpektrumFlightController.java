//Written by Dylan H. on 1/20/2023. Crab Drive superior, Mecanum Drive inferior.
//Edited by Anrew S. on 1/15/2025. We be swervin

package frc.lib;

import edu.wpi.first.hal.FRCNetComm.tResourceType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Handle input from flight controllers connected to the Driver Station.
 *
 * <p>
 * This class handles flight controller input that comes from the Driver
 * Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance
 * for each controller
 * and the mapping of ports to hardware buttons depends on the code in the
 * Driver Station.
 */

public class SpektrumFlightController extends GenericHID {
  
  private double RotateFactor = 1;
  private double DriveXFactor = 1;
  private double ThrottleFactor = 1;
  private double DriveYFactor = 1;
  private double KnobOffset = 1;
  private double UnknownFactor = 1;
  private double RightSliderFactor = 1;
  private double LeftSliderFactor = 1;
  private double xDeadBand = .1;
  private double yDeadBand = 0.1;
  private double rotationDeadBand = 0.1;
  private double RotateOffset = 0;
  private double DriveXOffset = 0;
  private double DriveYOffset = 0;
  private double ThrottleOffset = 0;

  // Gets the Factor for the x axis on the left stick, which controls a crab drive
  // bots rotation.
  public double getRotateFactor() {
    return RotateFactor;
  }

  // Sets the Factor for the x axis on the left stick, which controls a crab drive
  // bot's rotation.
  public void setRotateFactor(double rotateFactor) {
    RotateFactor = rotateFactor;
  }

  // Gets the Factor for the x axis on the rightt stick, which controls a crab
  // drive bots steering.
  public double getDriveXFactor() {
    return DriveXFactor;
  }

  // Sets the Factor for the x axis on the right stick, which controls a crab
  // drive bot's steering.
  public void setDriveXFactor(double driveXFactor) {
    DriveXFactor = driveXFactor;
  }

  // Gets the Factor for the y axis on the left stick, which controls a crab drive
  // bot's throttle.
  public double getThrottleFactor() {
    return ThrottleFactor;
  }

  // Sets the Factor for the y axis on the left stick, which controls a crab drive
  // bot's throttle.
  public void setThrottleFactor(double throttleFactor) {
    ThrottleFactor = throttleFactor;
  }

  // Gets the Factor for the y axis on the rightt stick, which controls a crab
  // drive bot's motion.
  public double getDriveYFactor() {
    return DriveYFactor;
  }

  // Sets the Factor for the y axis on the rightt stick, which controls a crab
  // drive bot's motion.
  public void setDriveYFactor(double driveYFactor) {
    DriveYFactor = driveYFactor;
  }

  // Gets the Factor for the axis on the knob, which controls something. Not sure
  // what yet.
  public double getKnobOffset() {
    return KnobOffset;
  }

  // Sets the Factor for the axis on the knob, which controls something. Not sure
  // what yet.
  public void setKnobOffset(double knobFactor) {
    KnobOffset = knobFactor;
  }

  // Gets the Factor for the axis that doesn't exist.
  public double getUnknownFactor() {
    return KnobOffset;
  }

  // Sets the Factor for the axis that doesn't exist.
  public void setUnknownFactor(double unknownFactor) {
    UnknownFactor = unknownFactor;
  }

  // Gets the Factor for the left slider.
  public double getLeftSliderset() {
    return LeftSliderFactor;
  }

  // Sets the Factor for the left slider.
  public void setLeftSliderFactor(double leftSliderFactor) {
    LeftSliderFactor = leftSliderFactor;
  }

  // Gets the Factor for the rght slider.
  public double getRightSliderset() {
    return RightSliderFactor;
  }

  // Sets the Factor for the rightt slider.
  public void setRightSliderFactor(double rightSliderFactor) {
    RightSliderFactor = rightSliderFactor;
  }

  public double getRotateOffset() {
    return RotateOffset;
  }

  public void setRotateOffset(double rotateOffset) {
    RotateOffset = rotateOffset;
  }

  public double getDriveXOffset() {
    return DriveXOffset;
  }

  public void setDriveXOffset(double driveXOffset) {
    DriveXOffset = driveXOffset;
  }

  public double getDriveYOffset() {
    return DriveYOffset;
  }

  public void setDriveYOffset(double driveYOffset) {
    DriveYOffset = driveYOffset;
  }

  public double getThrottleOffset() {
    return ThrottleOffset;
  }

  public void setThrottleOffset(double throttleOffset) {
    ThrottleOffset = throttleOffset;
  }

  public double getxDeadBand() {
    return xDeadBand;
  }

  public void setxDeadBand(double xDeadBand) {
    this.xDeadBand = xDeadBand;
  }

  public double getyDeadBand() {
    return yDeadBand;
  }

  public void setyDeadBand(double yDeadBand) {
    this.yDeadBand = yDeadBand;
  }

  public double getRotationDeadBand() {
    return rotationDeadBand;
  }

  public void setRotationDeadBand(double rotationDeadBand) {
    this.rotationDeadBand = rotationDeadBand;
  }

  /** Represents a digital button on an FlightController. */
  public enum Button {
    kA(1),
    kBBackward(2),
    kBForward(3),
    kCUp(4),
    kCDown(5),
    kDUp(6),
    kDDown(7),
    kFDown(8),
    kFUp(9),
    kGDown(10),
    kGUp(11),
    kH(12),
    kI(13),
    kReset(14),
    kCancel(15),
    kSelectPress(16),
    kSelectLeft(17),
    kSelectRight(18),
    kBelowLeftStickLeft(19),
    kBelowLeftStickRight(20),
    kBesidesLeftStickDown(21),
    kBesidesLeftStickUp(22),
    kBelowRightStickLeft(23),
    kBelowRightStickRight(24),
    kBesidesRightStickDown(25),
    kBesidesRightStickUp(26),
    kUnknownButton(27);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods.
     * This is done by
     * stripping the leading `k`, and if not a Bumper button append `Button`.
     *
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Bumper")) {
        return name;
      }
      return name + "Button";
    }
  }

  /** Represents an axis on an FlightController. */
  public enum Axis {
    kRotateAxis(0),
    kThrottle(1),
    kLeftSlider(2),
    kDriveX(3),
    kDriveY(4),
    kRightSlider(5),
    kUnknownAxis(6),
    kTopRightKnob(7);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This
     * is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is
   *             plugged into.
   */
  public SpektrumFlightController(final int port) {
    super(port);

    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }


  /**
   * Get the X axis value of the left stick of the controller, which controls
   * rotation.
   *
   * @return The axis value.
   */
  public double getRotate() {
    if (((getRawAxis(Axis.kRotateAxis.value) - RotateOffset) * RotateFactor) > 1) {
      return 1;
    }
    if (((getRawAxis(Axis.kRotateAxis.value) - RotateOffset) * RotateFactor) < -1) {
      return -1;
    }
    if ((((getRawAxis(Axis.kRotateAxis.value) - RotateOffset) * RotateFactor) > -rotationDeadBand)
        && ((getRawAxis(Axis.kRotateAxis.value) - RotateOffset) * RotateFactor) < rotationDeadBand) {
      return 0;
    }
    return (getRawAxis(Axis.kRotateAxis.value) - RotateOffset) * RotateFactor;

  }

  /**
   * Get the X axis value of the right stick of the controller, Which turns the
   * bot.
   *
   * @return The axis value.
   */
  public double getDriveX() {
    if (((getRawAxis(Axis.kDriveX.value) - DriveXOffset ) * DriveXFactor) > 1) {
      return 1;
    }
    if (((getRawAxis(Axis.kDriveX.value) - DriveXOffset ) * DriveXFactor) < -1) {
      return -1;
    }
    if ((((getRawAxis(Axis.kDriveX.value) - DriveXOffset ) * DriveXFactor) > -xDeadBand)
        && ((getRawAxis(Axis.kDriveX.value) - DriveXOffset ) * DriveXFactor) < xDeadBand) {
      return 0;
    }
    return (getRawAxis(Axis.kDriveX.value) - DriveXOffset ) * DriveXFactor;
  }

  /**
   * Get the throttle value of the left stick of the controller.
   *
   * @return The axis value.
   */
  public double getThrottle() {
    if (((getRawAxis(Axis.kThrottle.value) - ThrottleOffset) * ThrottleFactor) > 1) {
      return 1;
    }
    if (((getRawAxis(Axis.kThrottle.value) - ThrottleOffset) * ThrottleFactor) < -1) {
      return -1;
    }
    return (getRawAxis(Axis.kThrottle.value) - ThrottleOffset) * ThrottleFactor;
  }

  /**
   * Get the Y axis value of the right stick of the controller, which controls the
   * bots motion.
   *
   * @return The axis value.
   */
  public double getDriveY() {
    if (((getRawAxis(Axis.kDriveY.value) - DriveYOffset) * DriveYFactor) > 1) {
      return 1;
    }
    if (((getRawAxis(Axis.kDriveY.value) - DriveYOffset) * DriveYFactor) < -1) {
      return -1;
    }
    if ((((getRawAxis(Axis.kDriveY.value) - DriveYOffset) * DriveYFactor) > -yDeadBand)
        && ((getRawAxis(Axis.kDriveY.value) - DriveYOffset) * DriveYFactor) < yDeadBand) {
      return 0;
    }
    return (getRawAxis(Axis.kDriveY.value) - DriveYOffset) * DriveYFactor;
  }


      /**
   * Makes a deadband for the x axis
   *
   * @return The axis value.
   */
  public boolean driveXInDeadband() {
    if (((getRawAxis(Axis.kDriveX.value) * DriveXFactor) > -xDeadBand)
        && (getRawAxis(Axis.kDriveX.value) * DriveXFactor) < xDeadBand) {
      return true;

    }

    return false;

  }

    /**
   * Makes a deadband for the y axis
   *
   * @return The axis value.
   */
  public boolean driveYInDeadband() {
    if (((getRawAxis(Axis.kDriveY.value) * DriveYFactor) > -yDeadBand)
        && (getRawAxis(Axis.kDriveY.value) * DriveYFactor) < yDeadBand) {
      return true;

    }

    return false;

  }

      /**
   * Makes a deadband for the y axis
   *
   * @return The axis value.
   */
  public boolean driveRotateInDeadband() {
    if (((getRawAxis(Axis.kRotateAxis.value) * RotateFactor) > -rotationDeadBand)
        && (getRawAxis(Axis.kRotateAxis.value) * RotateFactor) < rotationDeadBand) {
      return true;

    }

    return false;

  }

  /**
   * Get the axis value of the knob on the top right of the controller.
   *
   * @return The axis value.
   */
  public double getUnkownAxis() {
    if ((getRawAxis(Axis.kUnknownAxis.value) * UnknownFactor) > 1) {
      return 1;
    }
    if ((getRawAxis(Axis.kUnknownAxis.value) * UnknownFactor) < -1) {
      return -1;
    }
    return getRawAxis(Axis.kUnknownAxis.value) * UnknownFactor;
  }

  /**
   * Get the axis value of the axis that the contoller doesn't have.
   *
   * @return The axis value.
   */
  public double getKnobAxis() {
    if ((getRawAxis(Axis.kTopRightKnob.value) * KnobOffset) > 1) {
      return 1;
    }
    if ((getRawAxis(Axis.kTopRightKnob.value) * KnobOffset) < -1) {
      return -1;
    }
    return getRawAxis(Axis.kTopRightKnob.value) * KnobOffset;
  }

  /**
   * Get the axis value of the slider on the back left of the controller.
   *
   * @return The axis value.
   */
  public double getLeftSliderAxis() {
    if ((getRawAxis(Axis.kLeftSlider.value) * LeftSliderFactor) > 1) {
      return 1;
    }
    if ((getRawAxis(Axis.kLeftSlider.value) * LeftSliderFactor) < -1) {
      return -1;
    }
    if (((getRawAxis(Axis.kLeftSlider.value) * LeftSliderFactor) > -0.08)
        && (getRawAxis(Axis.kLeftSlider.value) * LeftSliderFactor) < 0.08) {
      return 0;
    }
    return getRawAxis(Axis.kLeftSlider.value) * LeftSliderFactor;
  }

  /**
   * Get the axis value of the slider on the back Right of the controller.
   *
   * @return The axis value.
   */
  public double getRightSliderAxis() {
    if ((getRawAxis(Axis.kRightSlider.value) * RightSliderFactor) > 1) {
      return 1;
    }
    if ((getRawAxis(Axis.kRightSlider.value) * RightSliderFactor) < -1) {
      return -1;
    }
    if (((getRawAxis(Axis.kRightSlider.value) * RightSliderFactor) > -0.04)
        && (getRawAxis(Axis.kRightSlider.value) * RightSliderFactor) < 0.04) {
      return 0;
    }
    return getRawAxis(Axis.kRightSlider.value) * RightSliderFactor;
  }

  /**
   * Get the value of the A button.
   *
   * @return The button value.
   */
  public boolean getAButton() {
    return getRawButton(Button.kA.value);

  }

  /**
   * Get the value of the B button.
   * 
   * @param b
   *
   * @return The button value.
   */
  public boolean getBBackwardButton() {
    return getRawButton(Button.kBBackward.value);

  }

  /**
   * Get the value of the B button.
   *
   * @return The button value.
   */
  public boolean getBForwardButton() {
    return getRawButton(Button.kBForward.value);

  }

  /**
   * Get the value of the C button.
   *
   * @return The button value.
   */
  public boolean getCDownButton() {
    return getRawButton(Button.kCDown.value);

  }

  /**
   * Get the value of the C button.
   *
   * @return The button value.
   */
  public boolean getCUpButton() {
    return getRawButton(Button.kCUp.value);

  }

  /**
   * Get the value of the D button.
   *
   * @return The button value.
   */
  public boolean getDDownButton() {
    return getRawButton(Button.kDDown.value);

  }

  /**
   * Get the value of the D button.
   *
   * @return The button value.
   */
  public boolean getDUpButton() {
    return getRawButton(Button.kDUp.value);

  }

  /**
   * Get the value of the F button.
   *
   * @return The button value.
   */
  public boolean getFDownButton() {
    return getRawButton(Button.kFDown.value);

  }

  /**
   * Get the value of the F button.
   *
   * @return The button value.
   */
  public boolean getFUpButton() {
    return getRawButton(Button.kFUp.value);

  }

  /**
   * Get the value of the G button.
   *
   * @return The button value.
   */
  public boolean getGDownButton() {
    return getRawButton(Button.kGDown.value);

  }

  /**
   * Get the value of the G button.
   *
   * @return The button value.
   */
  public boolean getGUpButton() {
    return getRawButton(Button.kGUp.value);

  }

  /**
   * Get the value of the H button.
   *
   * @return The button value.
   */
  public boolean getHButton() {
    return getRawButton(Button.kH.value);

  }

  /**
   * Get the value of the I button.
   *
   * @return The button value.
   */
  public boolean getIButton() {
    return getRawButton(Button.kI.value);

  }

  /**
   * Get the value of the Reset button.
   *
   * @return The button value.
   */
  public boolean getResetButton() {
    return getRawButton(Button.kReset.value);

  }

  /**
   * Get the value of the Cancel button.
   *
   * @return The button value.
   */
  public boolean getCancelButton() {
    return getRawButton(Button.kCancel.value);

  }

  /**
   * Get the value of the Select button.
   *
   * @return The button value.
   */
  public boolean getSelectPressButton() {
    return getRawButton(Button.kSelectPress.value);

  }

  /**
   * Get the value of the Select button.
   *
   * @return The button value.
   */
  public boolean getSelectLeftButton() {
    return getRawButton(Button.kSelectLeft.value);

  }

  /**
   * Get the value of the Select button.
   *
   * @return The button value.
   */
  public boolean getSelectRightButton() {
    return getRawButton(Button.kSelectRight.value);

  }

  /**
   * Get the value of the Lower Left slider looking button.
   *
   * @return The button value.
   */
  public boolean getBelowLeftStickLeftButton() {
    return getRawButton(Button.kBelowLeftStickLeft.value);

  }

  /**
   * Get the value of the Lower Left slider looking button.
   *
   * @return The button value.
   */
  public boolean getBelowLeftStickRightButton() {
    return getRawButton(Button.kBelowLeftStickRight.value);

  }

  /**
   * Get the value of the Lower Right slider looking button.
   *
   * @return The button value.
   */
  public boolean getBelowRightStickRightButton() {
    return getRawButton(Button.kBelowRightStickRight.value);

  }

  /**
   * Get the value of the Lower Right slider looking button.
   *
   * @return The button value.
   */
  public boolean getBelowRightStickLeftButton() {
    return getRawButton(Button.kBelowRightStickLeft.value);

  }

  /**
   * Get the value of the Upper Left slider looking button.
   *
   * @return The button value.
   */
  public boolean getBesidesLeftStickDownButton() {
    return getRawButton(Button.kBesidesLeftStickDown.value);

  }

  /**
   * Get the value of the Upper Left slider looking button.
   *
   * @return The button value.
   */
  public boolean getBesidesLeftStickUpButton() {
    return getRawButton(Button.kBesidesLeftStickUp.value);

  }

  /**
   * Get the value of the Upper Right slider looking button.
   *
   * @return The button value.
   */
  public boolean getBesidesRightStickDownButton() {
    return getRawButton(Button.kBesidesRightStickDown.value);

  }

  /**
   * Get the value of the Upper Right slider looking button.
   *
   * @return The button value.
   */
  public boolean getBesidesRightStickUpButton() {
    return getRawButton(Button.kBesidesRightStickUp.value);

  }

  /**
   * Get the value of the button that doesn't exist.
   *
   * @return The button value.
   */
  public boolean getUnknownButton() {
    return getRawButton(Button.kUnknownButton.value);

  }

}