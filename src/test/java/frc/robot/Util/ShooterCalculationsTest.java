package frc.robot.Util;


import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;



public class ShooterCalculationsTest {

    private static final double EPS = 1e-2; // tolerance for floating-point comparisons

    @Test
    public void testLaunchDistance(){
        double targetDistance_in = 60.0; // example distance
        double shooterAngle_deg = 45; // example angle
        double initialVelocity = ShooterCalculations.calculateInitialVelocity(shooterAngle_deg, targetDistance_in);
        double expectedDistance = 2.805; // in meters
    
        // Assert that the expected distance is close to the target distance
        assertEquals(expectedDistance, initialVelocity, EPS);
    }

     @Test
    public void testAngleShot() {
        double targetDistance_in = 60.0; // example distance
        double angle = ShooterCalculations.calculateAngleShot(targetDistance_in);
        double expectedAngle = Math.toRadians(85.38143); // in radians (example expected angle)
        assertEquals(expectedAngle, angle, EPS); // expected angle in radians
    }
}