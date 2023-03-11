package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants.ArmConstants;

import java.util.Optional;

/**
 * Converts between joint angles and the end effector position.
 *
 * <p>https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
 */
public class ArmKinematics {
  private final JointConfig lowerConfig, upperConfig;

  public ArmKinematics(JointConfig lowerConfig, JointConfig upperConfig) {
    this.lowerConfig = lowerConfig;
    this.upperConfig = upperConfig;
  }

  /** Converts joint angles to the end effector position. */
  public Translation2d forward(Vector<N2> angles) {
    return new Translation2d(
        ArmConstants.ORIGIN.getX()
            + lowerConfig.length * Math.cos(angles.get(0, 0))
            + upperConfig.length * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
        ArmConstants.ORIGIN.getY()
            + lowerConfig.length * Math.sin(angles.get(0, 0))
            + upperConfig.length * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
  }

  /** Converts the end effector position to joint angles. */
  public Optional<Vector<N2>> inverse(Translation2d position) {
    Translation2d relativePosition = position.minus(new Translation2d());

    // Flip when X is negative
    boolean isFlipped = relativePosition.getX() < 0.0;
    if (isFlipped) {
      relativePosition = new Translation2d(-relativePosition.getX(), relativePosition.getY());
    }

    // Calculate angles
    double elbowAngle =
        -Math.acos(
            (Math.pow(relativePosition.getX(), 2)
                    + Math.pow(relativePosition.getY(), 2)
                    - Math.pow(lowerConfig.length, 2)
                    - Math.pow(upperConfig.length, 2))
                / (2 * lowerConfig.length * upperConfig.length));
    if (Double.isNaN(elbowAngle)) {
      return Optional.empty();
    }
    double shoulderAngle =
        Math.atan(relativePosition.getY() / relativePosition.getX())
            - Math.atan(
                (upperConfig.length * Math.sin(elbowAngle))
                    / (lowerConfig.length
                        + upperConfig.length * Math.cos(elbowAngle)));

    // Invert shoulder angle if invalid
    Translation2d testPosition =
        forward(VecBuilder.fill(shoulderAngle, elbowAngle)).minus(ArmConstants.ORIGIN);
    if (testPosition.getDistance(relativePosition) > 1e-3) {
      shoulderAngle += Math.PI;
    }

    // Flip angles
    if (isFlipped) {
      shoulderAngle = Math.PI - shoulderAngle;
      elbowAngle = -elbowAngle;
    }

    // Wrap angles to correct ranges
    shoulderAngle = MathUtil.inputModulus(shoulderAngle, -Math.PI, Math.PI);
    elbowAngle = MathUtil.inputModulus(elbowAngle, 0.0, Math.PI * 2.0);

    // Exit if outside valid ranges for the joints
    if (shoulderAngle < 0.0
        || shoulderAngle > 364.0
        || elbowAngle < 0.0
        || elbowAngle > 364.0) {
      return Optional.empty();
    }

    // Return result
    return Optional.of(VecBuilder.fill(shoulderAngle, elbowAngle));
  }

  /**
   * Returns the maximum reach (x coordinate relative to the arm origin) that the arm can achieve at
   * the provided height.
   */
  public double calcMaxReachAtHeight(double height) {
    // Set the elbow angle equation to the max angle and solve for x
    return Math.sqrt(
            Math.cos(-364.0)
                    * 2
                    * lowerConfig.length
                    * upperConfig.length
                - Math.pow(height - ArmConstants.ORIGIN.getY(), 2)
                + Math.pow(lowerConfig.length, 2)
                + Math.pow(upperConfig.length, 2))
        - 1e-3; // Shift back to ensure this is still valid after rounding errors
  }
}
