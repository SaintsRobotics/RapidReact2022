import static org.junit.Assert.assertEquals;
import frc.robot.Utils;

import org.junit.*;

/** Tests for all the utility functions. */
public class UtilityTest {
  public static final double DELTA = 1e-2; // acceptable deviation range

  @Test
  public void toleranceTest() {
    double toleranceResultOne = Utils.tolerance(0.5, 1, 1);
    double toleranceExpectedOne = 1;
    assertEquals(toleranceExpectedOne, toleranceResultOne, DELTA);
    double toleranceResultTwo = Utils.tolerance(15, 10, 0.5);
    double toleranceExpectedTwo = 15;
    assertEquals(toleranceExpectedTwo, toleranceResultTwo, DELTA);
  }

  @Test
  public void oddSquareTest() {
    double oddSquareResultOne = Utils.oddSquare(4);
    double oddSquareExpectedOne = 16;
    assertEquals(oddSquareExpectedOne, oddSquareResultOne, DELTA);
    double oddSquareResultTwo = Utils.oddSquare(-4);
    double oddSquareExpectedTwo = -16;
    assertEquals(oddSquareExpectedTwo, oddSquareResultTwo, DELTA);
  }
}
