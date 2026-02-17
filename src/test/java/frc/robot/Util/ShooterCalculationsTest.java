package frc.robot.Util;

import org.junit.jupiter.api.Test;

public class ShooterCalculationsTest {

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
