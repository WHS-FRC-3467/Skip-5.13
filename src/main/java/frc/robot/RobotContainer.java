// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.UpperArmPID;
import frc.robot.subsystems.claw.ClawDefault;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.HoldPosition;
import frc.robot.subsystems.arm.LowerArmPID;
import frc.robot.subsystems.arm.RunFromJoy;
import frc.robot.subsystems.arm.SetBothArms;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TeleopSwerve;
import frc.robot.subsystems.limelight.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(1);

  // private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClawSubsytem m_claw = new ClawSubsytem();
  // private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Constants.tuningMode){
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    else{
      DriverStation.silenceJoystickConnectionWarning(false);
    }
    // Configure the trigger bindings

    //new Pneumatics();
    configureBindings();
    // m_drive.setDefaultCommand(new TeleopSwerve(m_drive, 
    //                                           () -> m_driverController.getLeftY(), 
    //                                           () -> m_driverController.getLeftX(), 
    //                                           () -> m_driverController.getRightX(), 
    //                                           m_driverController.leftStick(),
    //                                           m_driverController.leftBumper(),
    //                                           m_driverController.rightBumper()));
    
    m_arm.setDefaultCommand(new RunFromJoy(m_arm, ()-> m_operatorController.getLeftY(), ()-> m_operatorController.getRightY()));
    m_claw.setDefaultCommand(new ClawDefault(m_claw, ()-> m_operatorController.getRightTriggerAxis(), () -> m_operatorController.getLeftTriggerAxis()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.start().whileTrue(new InstantCommand(m_drive::zeroGyro, m_drive));

    //Arm towards Battery
    // m_operatorController.x().onTrue((new LowerArmPID(m_arm, ArmConstants.LOWER_HORIZANTAL_FORWARD)));
    // //Arm straight up
    // m_operatorController.y().onTrue(new LowerArmPID(m_arm, ArmConstants.LOWER_VERTICAL));
    // //Arm towards PDH
    // m_operatorController.b().onTrue(new LowerArmPID(m_arm, ArmConstants.LOWER_HORIZANTAL_BACKWARD));

    // m_operatorController.a().onTrue(new SetBothArms(m_arm, ArmSetpoints.TEST_SETPOINT_LOWER, ArmSetpoints.TEST_SETPOINT_UPPER));
    m_operatorController.a().onTrue(new HoldPosition(m_arm));
    //Arm towards Battery
    // m_operatorController.x().onTrue((new UpperArmRioPID(m_arm, ArmConstants.UPPER_HORIZANTAL_FORWARD)));
    // //Arm straight up
    // m_operatorController.y().onTrue(new UpperArmRioPID(m_arm, ArmConstants.UPPER_VERTICAL));
    // //Arm towards PDH
    // m_operatorController.b().onTrue(new UpperArmRioPID(m_arm, ArmConstants.UPPER_HORIZANTAL_BACKWARD));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
