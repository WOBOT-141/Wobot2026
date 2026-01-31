package frc.robot.commands.StickParameters;

public class StickBParameters implements IStickParameters {
    double RotateFactor;
    double DriveXFactor;
    double DriveYFactor;
    double ThrottleFactor;
    double RotateOffset;
    double DriveXOffset;
    double DriveYOffset;
    double ThrottleOffset;
    double KnobOffset;
    double deadbandX;
    double deadbandY;
    double deadbandRot;

    public StickBParameters() {
        RotateFactor = 1.35;
        DriveXFactor = 1.26;
        DriveYFactor = 1.23;
        ThrottleFactor = 1.55;
        RotateOffset = 0.031890;
        DriveXOffset = -0.03373;
        DriveYOffset = 0.024772;
        ThrottleOffset = 0;
        deadbandRot = 0.2;
        deadbandX = 0.2;
        deadbandY = 0.2;
        KnobOffset = 1;
    }

    @Override
    public double get_RotateFactor() {
        return RotateFactor;
    }

    @Override
    public double get_DriveXFactor() {
        return DriveXFactor;
    }

    @Override
    public double get_DriveYFactor() {
        return DriveYFactor;
    }

    @Override
    public double get_ThrottleFactor() {
        return ThrottleFactor;
    }

    @Override
    public double get_RotateOffset() {
        return RotateOffset;
    }

    @Override
    public double get_DriveXOffset() {
        return DriveXOffset;
    }

    @Override
    public double get_DriveYOffset() {
        return DriveYOffset;
    }

    @Override
    public double get_ThrottleOffset() {
        return ThrottleOffset;
    }

    @Override
    public double get_knobOffset() {
        return KnobOffset;
    }

    @Override
    public double get_deadbandX() {
        return deadbandX;
    }

    @Override
    public double get_deadbandY() {
        return deadbandY;
    }

    @Override
    public double get_deadbandRot() {
        return deadbandRot;
    }
}
