// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractToStowed extends SequentialCommandGroup {
  /** Creates a new RetractToStowed. */
  public RetractToStowed(ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Retracting from grid
    // if(arm.getSetpoint().equals(ArmSetpoints.MID_NODE) || arm.getSetpoint().equals(ArmSetpoints.MID_NODE_PLACED) || arm.getSetpoint() == ArmSetpoints.TOP_NODE || arm.getSetpoint() == ArmSetpoints.TOP_NODE_PLACED){
    //       Setpoint intermediateSetpoint = new Setpoint(ArmSetpoints.INTERMEDIATE_LOWER_POSITION, arm.getSetpoint().upperCone * 0.5, arm.getSetpoint().wristCone, 
    //                                                   ArmSetpoints.INTERMEDIATE_LOWER_POSITION, arm.getSetpoint().lowerCube * 0.5, arm.getSetpoint().wristCube);
    //       addCommands(
    //         new InstantCommand(()-> arm.updateAllSetpoints(intermediateSetpoint)),
    //         new WaitCommand(1.0),
    //         new InstantCommand( ()-> arm.updateAllSetpoints(ArmSetpoints.STOWED)),
    //         new WaitCommand(1.0)
    //       );
    // }
    // //Retracting from floor
    // else if (arm.getSetpoint() == ArmSetpoints.FLOOR){
    //   Setpoint intermediete = new Setpoint(ArmSetpoints.STOWED.lowerCone, ArmSetpoints.STOWED.upperCone + 20 , true, 
    //                                       ArmSetpoints.STOWED.lowerCube, ArmSetpoints.STOWED.upperCube + 20, true);
    //   addCommands(
    //         new InstantCommand( ()-> arm.updateAllSetpoints(intermediete)),
    //         new WaitCommand(2.0),
    //         new InstantCommand(()-> arm.updateAllSetpoints(ArmSetpoints.STOWED))
    //   );
    // }
    // //Retracting from Substation 
    // else if(arm.getSetpoint() == ArmSetpoints.SUBSTATION){
    //   addCommands(new InstantCommand(()-> arm.updateAllSetpoints(ArmSetpoints.STOWED)));
    // }
    // //Other
    // else{
      addCommands(new InstantCommand(()-> arm.updateAllSetpoints(ArmSetpoints.STOWED)));
    //}
  }
}
