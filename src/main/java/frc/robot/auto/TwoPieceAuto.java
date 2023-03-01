// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToMidNode;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw, LimelightSubsystem limelight) {
    PathPlannerTrajectory path1 = PathPlanner.loadPath("TwoPiecePart1", new PathConstraints(7.0, 8.5));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("TwoPiecePart2", new PathConstraints(2.0, 2.0));
    PathPlannerTrajectory path3 = PathPlanner.loadPath("TwoPiecePart3", new PathConstraints(7.0, 8.5));
    PathPlannerTrajectory path4 = PathPlanner.loadPath("TwoPiecePart4", new PathConstraints(4.5, 5.5));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cone)),
        new WaitCommand(0.03),
        new GoToPositionWithIntermediate(arm, ArmSetpoints.TOP_NODE),
        Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED_AND_OPEN)),
        Commands.runOnce(() -> GamePiece.setGamePiece(GamePieceType.Cube)),
        drive.followTrajectoryCommand(path1, true),
        drive.followTrajectoryCommand(path2, false).raceWith(Commands.runOnce(() -> arm.updateAllSetpoints(ArmSetpoints.FLOOR))).raceWith(Commands.run(()-> claw.driveClaw(0.8), claw)),
        new WaitCommand(0.03),
        drive.followTrajectoryCommand(path3, false).raceWith(new RetractToStowed(arm)),
        drive.followTrajectoryCommand(path4, false),
        new GoToMidNode(arm),
        new RetractToStowed(arm).raceWith(Commands.run(()-> claw.driveClaw(-0.5)))
    );
  }
}
