// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.arm.ScoreAndRetract;
import frc.robot.subsystems.arm.ScoreOnGrid;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiece extends SequentialCommandGroup {
  /** Creates a new TwoPieceV2. */
  public TwoPiece(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw) {
    PathPlannerTrajectory twoPiecePath = PathPlanner.loadPath("TwoPiece", new PathConstraints(2.0, 4.0));
    HashMap<String, Command> eventMap = new HashMap<>();
 
    eventMap.put("Retract", new SequentialCommandGroup(
                              new WaitCommand(0.25),
                              new RetractToStowed(arm),
                              Commands.run(()->claw.driveClaw(0.25)),
                              Commands.print("retract")).withTimeout(1.0));

    eventMap.put("Retract floor", new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new RetractToStowed(arm),
                                Commands.run(()->claw.driveClaw(0.25)),
                                Commands.print("retract floor")).withTimeout(1.0));

    eventMap.put("Floor position", new ParallelCommandGroup(
                                  Commands.print("Floor"),
                                  Commands.runOnce(()-> arm.updateAllSetpoints(ArmSetpoints.FLOOR)), 
                                  Commands.run(()->claw.driveClaw(1.0))).withTimeout(3.0));
    eventMap.put("Prep score", new ParallelCommandGroup(
                                              new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
                                              Commands.print("Prep score")));
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
      new WaitCommand(0.02),
      new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
      new ScoreOnGrid(arm, claw),      
      Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cube)),
      Commands.print(eventMap.toString()),
      new FollowPathWithEvents(drive.followTrajectoryCommand(twoPiecePath, true), 
                              twoPiecePath.getMarkers(), 
                              eventMap),     
      new ScoreAndRetract(arm, claw)
    );
  }
}

