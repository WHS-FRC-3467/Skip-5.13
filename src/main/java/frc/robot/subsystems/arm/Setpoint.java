// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Setpoint {
    public double lowerCone;
    public double upperCone;
    public boolean wristCone;
    public double lowerCube;
    public double upperCube;
    public boolean wristCube;
    public ArmState state;
    public ClawState clawCone;
    public ClawState clawCube;
    public Translation2d cubeEndEffector;
    public Translation2d coneEndEffector;
   
    public Setpoint(double lowerCone, double upperCone, boolean wristCone, ClawState clawCone, double lowerCube, double upperCube,
            boolean wristCube, ClawState clawCube, ArmState state) {
        this.lowerCone = lowerCone;
        this.upperCone = upperCone;
        this.wristCone = wristCone;
        this.clawCone = clawCone;
        this.lowerCube = lowerCube;
        this.upperCube = upperCube;
        this.wristCube = wristCube;
        this.clawCube = clawCube;
        this.state = state;
    }

    public Setpoint(Translation2d coneEndEffector, boolean wristCone, ClawState clawCone, Translation2d cubeEndEffector,
            boolean wristCube, ClawState clawCube, ArmState state) {
        this.wristCone = wristCone;
        this.clawCone = clawCone;
        this.wristCube = wristCube;
        this.clawCube = clawCube;
        this.state = state;
        this.cubeEndEffector = cubeEndEffector;
        this.coneEndEffector = coneEndEffector;
    }
    public enum ArmState{
        STOWED, FLOOR, MID_NODE, MID_NODE_PLACED, TOP_NODE, TOP_NODE_PLACED, SUBSTATION, INTERMEDIATE, OTHER
    }
    public enum ClawState{
        IN, OUT
    }
}
