// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceV2 extends SequentialCommandGroup {
  /** Creates a new ThreePieceAuto. */
  public ThreePieceV2(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw) {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("TwoPiecePart1v2", new PathConstraints(2.5, 4.0));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("TwoPiecePart2v2", new PathConstraints(3.5, 4.0));
    PathPlannerTrajectory path3 = PathPlanner.loadPath("TwoPiecePart3v2", new PathConstraints(4.0, 6.0));
    PathPlannerTrajectory path4 = PathPlanner.loadPath("TwoPiecePart4v2", new PathConstraints(2.5, 4.0));
    PathPlannerTrajectory path5 = PathPlanner.loadPath("ThreePiecePart5v2", new PathConstraints(2.5, 4.0));
    PathPlannerTrajectory path6 = PathPlanner.loadPath("ThreePiecePart6v2", new PathConstraints(4.0, 4.0));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
        new WaitCommand(0.02),
        new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
        Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED)),
        new WaitCommand(0.02),
        new WaitUntilCommand(arm::bothJointsAtSetpoint),
        Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED_AND_OPEN)),
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cube)),
        new WaitUntilCommand(arm::bothJointsAtSetpoint),
        drive.followTrajectoryCommand(path1, true).alongWith(new RetractToStowed(arm)),
        new ParallelCommandGroup(
          drive.followTrajectoryCommand(path2, false).raceWith(Commands.run(()-> claw.driveClaw(0.8), claw)),
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.FLOOR))
          )
        ),
        new WaitCommand(0.02),
        new ParallelDeadlineGroup(
          drive.followTrajectoryCommand(path3, false),
          new RetractToStowed(arm)
        ),
        new ParallelDeadlineGroup(
          drive.followTrajectoryCommand(path4, false),
          new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE)
        ),
        Commands.run(()-> claw.driveClaw(-0.5)).withTimeout(0.1),
        new ParallelDeadlineGroup(
          drive.followTrajectoryCommand(path5, false),
          new RetractToStowed(arm).raceWith(Commands.run(()-> claw.driveClaw(-0.5)))
        ),
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
        new ParallelDeadlineGroup(
          drive.followTrajectoryCommand(path6, false).raceWith(Commands.run(()-> claw.driveClaw(1.0), claw)),
          new SequentialCommandGroup(
            new WaitCommand(0.25),
            Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.FLOOR))
          )
        ),
        new RetractToStowed(arm).raceWith(Commands.run(()-> claw.driveClaw(0.2), claw))
      );
  }
}
