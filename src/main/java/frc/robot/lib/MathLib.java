package frc.robot.lib;

public class MathLib {
 /**
   * Normalizes the angle between 0 and 360 degrees.
   * @param angle the angle which needs to be normalized.
   * @return
   */
  public static double normalizeDegrees(double angle) {
    while (angle > 360) {
      angle -= 360;  
    }
    while (angle < 0) {
      angle += 360;
    }

    return angle;
  }   
}
