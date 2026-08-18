package frc.robot.commands.StickParameters;

public class StickBParameters implements IStickParameters {

    private AxisParameters leftXAxisParams;
    private AxisParameters leftYAxisParams;
    private AxisParameters rightXAxisParams;
    private AxisParameters rightYAxisParams;

    public StickBParameters() {
        leftXAxisParams  = new AxisParameters(-0.82, 0.02, 0.85, 0.2);
        leftYAxisParams  = new AxisParameters(-0.73, 0.04, 0.65, 0.0);
        rightXAxisParams = new AxisParameters(-0.89, -.04, 0.81, 0.2);
        rightYAxisParams = new AxisParameters(-0.89, 0.02, 0.9, 0.2);
    }

    @Override
    public AxisParameters getLeftXAxisParams() {
        return leftXAxisParams;
    }

    @Override
    public AxisParameters getLeftYAxisParams() {
        return leftYAxisParams;
    }

    @Override
    public AxisParameters getRightXAxisParams() {
        return rightXAxisParams;
    }

    @Override
    public AxisParameters getRightYAxisParams() {
        return rightYAxisParams;
    }

}