package frc.robot.Util;

public class ShotSolution {

    public final double angle_deg;
    public final double velocity_IPS;

    public ShotSolution(double angle_deg, double velocity_IPS) {
        this.angle_deg = angle_deg;
        this.velocity_IPS = velocity_IPS;
    }

    @Override
    public String toString() {
        return String.format("Angle: %.2f deg, Velocity: %.2f ips",
                angle_deg, velocity_IPS);
    }
}