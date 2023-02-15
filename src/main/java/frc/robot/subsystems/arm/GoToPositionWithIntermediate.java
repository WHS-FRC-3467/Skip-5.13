// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.Setpoint.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPositionWithIntermediate extends SequentialCommandGroup {
  /** Creates a new GoToPositionWithIntermediate. */
  public GoToPositionWithIntermediate(ArmSubsystem arm, Setpoint setpoint) {

    Setpoint intermediateSetpoint = new Setpoint(ArmSetpoints.INTERMEDIATE_LOWER_POSITION, setpoint.upperCone * 0.5, setpoint.wristCone, 
                                                ArmSetpoints.INTERMEDIATE_LOWER_POSITION, (setpoint.upperCube) * 0.5, setpoint.wristCube, ArmState.INTERMEDIATE);
    addCommands(  
        new InstantCommand(()-> arm.updateAllSetpoints(intermediateSetpoint)),
        new WaitUntilCommand(()-> arm.bothJointsAtSetpoint()),
        new InstantCommand( ()-> arm.updateAllSetpoints(setpoint)),
        new WaitUntilCommand(()-> arm.bothJointsAtSetpoint()));
  }
}
