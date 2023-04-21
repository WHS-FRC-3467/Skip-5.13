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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.arm.ScoreOnGrid;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.cubeShooter.CubeShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceTryThree extends SequentialCommandGroup {
  /** Creates a new ThreePieceTry3. */
  public ThreePieceTryThree(ArmSubsystem arm, DriveSubsystem drive, ClawSubsytem claw, CubeShooterSubsystem cuby) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory path1 = PathPlanner.loadPath("ThreePieceCubyPart1", new PathConstraints(3.0, 6.0));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("ThreePieceCubyPart2", new PathConstraints(4.0, 6.0));
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMap2 = new HashMap<>();
    eventMap.put("Retract", 
                              new RetractToStowed(arm).alongWith(
                               new WaitCommand(1.4).andThen(
                                Commands.runOnce(cuby::deployShooter, cuby).andThen(
                                                        Commands.run(()->cuby.conditionalRun(0.4), cuby))                        
                              )
                            ).withTimeout(3.0));
    eventMap.put("Retract Cuby", Commands.runOnce(cuby::retractShooter).alongWith(Commands.run(()->cuby.shoot(0.0))));

    eventMap2.put("Retract", new ParallelCommandGroup(
                                        Commands.runOnce(cuby::deployShooter),
                                        Commands.run(()->cuby.conditionalRun(0.4))).withTimeout(5.0));
    eventMap2.put("Retract Cuby", Commands.runOnce(cuby::retractShooter).alongWith(Commands.run(()->cuby.shoot(0.0))));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
        new WaitCommand(0.02),
        new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
        new ScoreOnGrid(arm, claw),
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cube)),
        new WaitUntilCommand(arm::bothJointsAtSetpoint),
        new FollowPathWithEvents(drive.followTrajectoryCommand(path1, true), path1.getMarkers(), eventMap),
        new WaitCommand(0.02),
        Commands.run(()-> cuby.shoot(-0.8)).withTimeout(0.5),
        new WaitCommand(0.02),
        new FollowPathWithEvents(drive.followTrajectoryCommand(path2, false), path2.getMarkers(), eventMap2).withTimeout(5.5),
        Commands.run(()-> cuby.shoot(-0.4), cuby)
      );  
  }
}
