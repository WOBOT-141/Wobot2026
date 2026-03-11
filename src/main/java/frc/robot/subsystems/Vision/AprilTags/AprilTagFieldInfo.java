package frc.robot.subsystems.Vision.AprilTags;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagFieldInfo {

    public AprilTagFieldInfo(int id, int pairedId, Alliance alliance, FieldElement fieldElement, double xToElement, double yToElement, double zToElement){
        Id = id;
        PairedId = pairedId;
        Alliance = alliance;
        FieldElement = fieldElement;
        XToElement = xToElement;
        YToElement = yToElement;
        ZToElement = zToElement;
    }

    public final int Id;

    public final int PairedId;

    public final Alliance Alliance;

    public final FieldElement FieldElement;

    public final double XToElement;

    public final double YToElement;

    public final double ZToElement;
}