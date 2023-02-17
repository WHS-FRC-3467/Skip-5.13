// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto;

// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.arm.ArmSubsystem;
// import frc.robot.subsystems.claw.ClawSubsytem;
// import frc.robot.subsystems.drive.DriveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TwoPieceAuto extends SequentialCommandGroup {
//   /** Creates a new TwoPieceAuto. */
//   public TwoPieceAuto(DriveSubsystem drive, ArmSubsystem arm, ClawSubsytem claw) {
//     List<PathPlannerTrajectory> path1 = PathPlanner.loadPathGroup("TwoPieceAuto", 
//                                                                       new PathConstraints(4, 4),
//                                                                       new PathConstraints(1, 1),
//                                                                       new PathConstraints(4, 4));
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(

//     );
//   }
// }
