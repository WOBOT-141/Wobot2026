package frc.robot.Util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.BiFunction;
import java.util.function.Function;

public class TestHelper {

    private static final double EPS = 1e-2;

    /** 
     * Run a test with two inputs and on expected output.
     * @param method the method to test, as a BiFunction that takes two Doubles and returns a Double
     * @param input1 the first input to the method
     * @param input2 the second input to the method
     * @param expected the expected output of the method
     */
    public static void runTest(BiFunction<Double, Double, Double> method,
                               double input1,
                               double input2,
                               double expected) {

        double result = method.apply(input1, input2);
        assertEquals(expected, result, EPS);
    }

        /** 
        * Run a test with one input and on expected output
        called using the format TestHelper.runTest(Class::Method, input, expected);
        * @param method the method to test, as a Function that takes a Double and returns a Double
        * @param input the input to the method
        * @param expected the expected output of the method
        */
    public static void runTest(Function<Double, Double> method,
                               double input,
                               double expected) {

        double result = method.apply(input);
        assertEquals(expected, result, EPS);
    }
}
