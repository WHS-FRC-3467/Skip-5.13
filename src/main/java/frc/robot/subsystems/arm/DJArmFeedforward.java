// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 * Adapted from 6328 ArmFeedforward class
 */
public class DJArmFeedforward {
  private static final double g = 9.80665;
  
  private final JointConfig joint_Upper;
  private final JointConfig joint_Lower;

  public DJArmFeedforward(JointConfig joint_Upper, JointConfig joint_Lower) {
    this.joint_Upper = joint_Upper;
    this.joint_Lower = joint_Lower;
  }

  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var M = new Matrix<>(N2.instance, N2.instance);
    var C = new Matrix<>(N2.instance, N2.instance);
    var Tg = new Matrix<>(N2.instance, N1.instance);

    M.set(
        0,
        0,
        joint_Upper.mass * Math.pow(joint_Upper.cgRadius, 2.0)
            + joint_Lower.mass * (Math.pow(joint_Upper.length, 2.0) + Math.pow(joint_Lower.cgRadius, 2.0))
            + joint_Upper.moi
            + joint_Lower.moi
            + 2
                * joint_Lower.mass
                * joint_Upper.length
                * joint_Lower.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_Lower.mass * Math.pow(joint_Lower.cgRadius, 2.0)
            + joint_Lower.moi
            + joint_Lower.mass
                * joint_Upper.length
                * joint_Lower.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_Lower.mass * Math.pow(joint_Lower.cgRadius, 2.0)
            + joint_Lower.moi
            + joint_Lower.mass
                * joint_Upper.length
                * joint_Lower.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_Lower.mass * Math.pow(joint_Lower.cgRadius, 2.0) + joint_Lower.moi);
    C.set(
        0,
        0,
        -joint_Lower.mass
            * joint_Upper.length
            * joint_Lower.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_Lower.mass
            * joint_Upper.length
            * joint_Lower.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_Lower.mass
            * joint_Upper.length
            * joint_Lower.cgRadius
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (joint_Upper.mass * joint_Upper.cgRadius + joint_Lower.mass * joint_Upper.length)
                * g
                * Math.cos(position.get(0, 0))
            + joint_Lower.mass
                * joint_Lower.cgRadius
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_Lower.mass
            * joint_Lower.cgRadius
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        joint_Upper.motor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_Lower.motor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}