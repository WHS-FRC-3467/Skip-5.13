// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class Setpoint {
    public double m_lowerCone;
    public double m_upperCone;
    public boolean wristCone;
    public double m_lowerCube;
    public double m_upperCube;
    public boolean wristCube;
    /**
     * 
     * @param m_lowerCone
     * @param m_upperCone
     * @param wristCone
     * @param m_lowerCube
     * @param m_upperCube
     * @param wristCube
     */
    public Setpoint(double m_lowerCone, double m_upperCone, boolean wristCone, double m_lowerCube, double m_upperCube,
            boolean wristCube) {
        this.m_lowerCone = m_lowerCone;
        this.m_upperCone = m_upperCone;
        this.wristCone = wristCone;
        this.m_lowerCube = m_lowerCube;
        this.m_upperCube = m_upperCube;
        this.wristCube = wristCube;
    }
}
