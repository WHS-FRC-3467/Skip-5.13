// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class Setpoint {
    public double lowerCone;
    public double upperCone;
    public boolean wristCone;
    public double lowerCube;
    public double upperCube;
    public boolean wristCube;
    public ArmState state;
    /**
     * 
     * @param m_lowerCone
     * @param m_upperCone
     * @param wristCone
     * @param m_lowerCube
     * @param m_upperCube
     * @param wristCube
     */
    public Setpoint(double lowerCone, double upperCone, boolean wristCone, double lowerCube, double upperCube,
            boolean wristCube, ArmState state) {
        this.lowerCone = lowerCone;
        this.upperCone = upperCone;
        this.wristCone = wristCone;
        this.lowerCube = lowerCube;
        this.upperCube = upperCube;
        this.wristCube = wristCube;
        this.state = state;
    }
    public enum ArmState{
        STOWED, FLOOR, MID_NODE, MID_NODE_PLACED, TOP_NODE, TOP_NODE_PLACED, SUBSTATION, INTERMEDIATE, OTHER
    }
}
