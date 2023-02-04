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
  
  private final JointConfig Joint_Lower;
  private final JointConfig joint_Upper;

  public DJArmFeedforward(JointConfig Joint_Lower, JointConfig joint_Upper) {
    this.Joint_Lower = Joint_Lower;
    this.joint_Upper = joint_Upper;
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
        Joint_Lower.mass * Math.pow(Joint_Lower.cgRadius, 2.0)
            + joint_Upper.mass * (Math.pow(Joint_Lower.length, 2.0) + Math.pow(joint_Upper.cgRadius, 2.0))
            + Joint_Lower.moi
            + joint_Upper.moi
            + 2
                * joint_Upper.mass
                * Joint_Lower.length
                * joint_Upper.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_Upper.mass * Math.pow(joint_Upper.cgRadius, 2.0)
            + joint_Upper.moi
            + joint_Upper.mass
                * Joint_Lower.length
                * joint_Upper.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_Upper.mass * Math.pow(joint_Upper.cgRadius, 2.0)
            + joint_Upper.moi
            + joint_Upper.mass
                * Joint_Lower.length
                * joint_Upper.cgRadius
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_Upper.mass * Math.pow(joint_Upper.cgRadius, 2.0) + joint_Upper.moi);
    C.set(
        0,
        0,
        -joint_Upper.mass
            * Joint_Lower.length
            * joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_Upper.mass
            * Joint_Lower.length
            * joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_Upper.mass
            * Joint_Lower.length
            * joint_Upper.cgRadius
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (Joint_Lower.mass * Joint_Lower.cgRadius + joint_Upper.mass * Joint_Lower.length)
                * g
                * Math.cos(position.get(0, 0))
            + joint_Upper.mass
                * joint_Upper.cgRadius
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_Upper.mass
            * joint_Upper.cgRadius
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        Joint_Lower.motor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_Upper.motor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}