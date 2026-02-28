package frc.robot.Util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

public class ShooterCalculationsTest {

    private static final double EPS = 1e-2; // tolerance for floating-point comparisons

    @Test
    public void testLaunchDistance() {
        double targetDistance_in = 60.0; // example distance
        double shooterAngle_deg = 45; // example angle
        double initialVelocity = ShooterCalculations.getLaunchSpeed(shooterAngle_deg, targetDistance_in);
        double expectedVelocity = 2.805; // in meters

        // Assert that the expected distance is close to the target distance
        assertEquals(expectedVelocity, initialVelocity, EPS);
    }

    @Test
    public void testAngleShot() {
        double targetDistance_in = 60.0; // example distance
        double targetHeight_in = 45;
        double angle = ShooterCalculations.getLaunchAngle(targetDistance_in, targetHeight_in);
        double expectedAngle = Math.toRadians(85.38143); // in radians (example expected angle)
        assertEquals(expectedAngle, angle, EPS); // expected angle in radians
    }

    @Test
    public void testAngleAngle() {
        double targetDistance_in = 20.0; // example distance
           double targetHeight_in = 45;
        double angle = ShooterCalculations.getLaunchAngle(targetDistance_in, targetHeight_in);
        double expectedAngle = Math.toRadians(65); // example expected angle
        assertEquals(expectedAngle, angle, EPS, "mar mar in your computer.");
    }

}
    @Test
    void testInitialVelocity() {
        TestHelper.runTest(
            ShooterCalculations::calculateInitialVelocity,
            45.0,     // angle (deg)
            60.0,     // distance (in)
            2.805     // expected velocity (m/s)
        );
    }

    @Test
    void testAngleShot() {
        TestHelper.runTest(
            ShooterCalculations::calculateAngleShot,
            60.0,
            Math.toRadians(85.38143)
        );
    }
    
}
