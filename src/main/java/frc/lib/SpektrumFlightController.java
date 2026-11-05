//Written by Dylan H. on 1/20/2023. Crab Drive superior, Mecanum Drive inferior.
//Edited by Anrew S. on 1/15/2025. We be swervin

package frc.lib;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.StickParameters.AxisParameters;
import frc.robot.commands.StickParameters.IStickParameters;

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


  private IStickParameters _stickParameters;
  
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is
   *             plugged into.
   */
  public SpektrumFlightController(final int port, final IStickParameters stickParameters) {
    super(port);

    _stickParameters = stickParameters;
    HAL.report(tResourceType.kResourceType_XboxController, port + 1);
  }


  private double getCorrectedAxisInput(double axisInput, AxisParameters params) {
    double corrected = 0;
  
    if (axisInput > params.axisMid){
      corrected = (axisInput - params.axisMid) / Math.abs(params.axisMax - params.axisMid);
    } else if (axisInput < params.axisMid){
      corrected = (axisInput - params.axisMid) / Math.abs(params.axisMin - params.axisMid);
    } else 
      return 0;

    return Math.min(Math.max(corrected, -1), 1);
  }


  private double applyContinuousDeadband(double axisInput, double deadband) {
    if (Math.abs(axisInput) < deadband)
      return 0;

    double factor = 1/(1-deadband);
    return (axisInput * factor) + Math.copySign(deadband, axisInput);
  }


  private double applyStepDeadband(double axisInput, double deadband) {
    if (Math.abs(axisInput) < deadband)
      return 0;

    return axisInput;
  }

  /**
   * Get the X axis value of the left stick of the controller, which controls
   * rotation.
   *
   * @return The axis value.
   */
  public double getRotate() {
    double raw = getRawAxis(Axis.kRotateAxis.value);
    double corrected = getCorrectedAxisInput(raw, _stickParameters.getLeftXAxisParams());
    return applyStepDeadband(corrected, _stickParameters.getLeftXAxisParams().axisDeadband);
  }

  /**
   * Get the throttle value of the left stick of the controller.
   *
   * @return The axis value.
   */
  public double getThrottle() {
    double raw = getRawAxis(Axis.kThrottle.value);
    return getCorrectedAxisInput(raw, _stickParameters.getLeftYAxisParams());
  }

  /**
   * Get the X axis value of the right stick of the controller, which turns the
   * bot.
   *
   * @return The axis value.
   */

    public double getAngleDriveMagnitude() {
      double rawX = getRawAxis(Axis.kDriveX.value);
      double correctedX = getCorrectedAxisInput(rawX, _stickParameters.getRightXAxisParams());
      double rawY = getRawAxis(Axis.kDriveY.value);
      double correctedY = getCorrectedAxisInput(rawY, _stickParameters.getRightYAxisParams());
      double angleDriveMagnitude = Math.sqrt(Math.pow(correctedX, 2) + Math.pow(correctedY, 2));
      return angleDriveMagnitude;
      // applyStepDeadband(angleDriveMagnitude, _stickParameters.getRightYAxisParams().axisDeadband);
    }

  public double getDriveX() {
    double rawX = getRawAxis(Axis.kDriveX.value);
    double correctedX = getCorrectedAxisInput(rawX, _stickParameters.getRightXAxisParams());
    double rawY = getRawAxis(Axis.kDriveY.value);
    double correctedY = getCorrectedAxisInput(rawY,  _stickParameters.getRightYAxisParams());
    if(Math.sqrt(Math.pow(correctedX, 2) + Math.pow(correctedY, 2)) < _stickParameters.getRightXAxisParams().axisDeadband){
      return 0;
    }
    else{
      return correctedX;
    }
    //TODO add to y
    //return applyStepDeadband(correctedX,  _stickParameters.getRightXAxisParams().axisDeadband);
   // return applyStepDeadband(Math.sqrt(Math.pow(correctedX, 2) + Math.pow(correctedY, 2)), _stickParameters.getRightXAxisParams().axisDeadband);
   //return corrected;
  }

  /**
   * Get the Y axis value of the right stick of the controller, which controls the
   * bots motion.
   *
   * @return The axis value.
   */
  public double getDriveY() {
    double rawY = getRawAxis(Axis.kDriveY.value);
    double corrected = getCorrectedAxisInput(rawY, _stickParameters.getRightYAxisParams());

    return applyStepDeadband(corrected, _stickParameters.getRightYAxisParams().axisDeadband);
    //return corrected;
  }

  
  

  /**
   * Get the axis value of the axis that the contoller doesn't have.
   *
   * @return The axis value.
   */
  public double getKnobAxis() {
    return getRawAxis(Axis.kTopRightKnob.value);
  }

  /**
   * Get the axis value of the knob on the top right of the controller.
   *
   * @return The axis value.
   */
  public double getUnknownAxis() {
    return getRawAxis(Axis.kUnknownAxis.value);
  }

  /**
   * Get the axis value of the slider on the back left of the controller.
   *
   * @return The axis value.
   */
  public double getLeftSliderAxis() {
    return getRawAxis(Axis.kLeftSlider.value);
  }

  /**
   * Get the axis value of the slider on the back Right of the controller.
   *
   * @return The axis value.
   */
  public double getRightSliderAxis() {
    return getRawAxis(Axis.kRightSlider.value);
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