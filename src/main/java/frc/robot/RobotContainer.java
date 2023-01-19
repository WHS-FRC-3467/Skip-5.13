// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.RunArmFromDashboard;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TeleopSwerve;

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

  // private final CommandXboxController m_operatorController =
  //     new CommandXboxController(1);

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  private final SendableChooser<String> m_armModeChooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    new Pneumatics();
    configureBindings();
    m_drive.setDefaultCommand(new TeleopSwerve(m_drive, 
                                              () -> m_driverController.getLeftX(), 
                                              () -> m_driverController.getLeftY(), 
                                              () -> m_driverController.getRightX(), 
                                              m_driverController.leftStick(),
                                              m_driverController.leftBumper(),
                                              m_driverController.rightBumper()));
    //"LowerPID", "LowerMM", "UpperPID", "UpperMM"
    if(Constants.tuningMode){
      m_armModeChooser.addOption("Lower Joint PID", "LowerPID");
      m_armModeChooser.addOption("Upper Joint PID", "UpperPID");
      m_armModeChooser.addOption("Lower Joint Motion Magic", "LowerMM");
      m_armModeChooser.addOption("Upper Joint Motion Magic", "UpperMM");
      m_armModeChooser.setDefaultOption("Lower Joint PID", "LowerPID");
      SmartDashboard.putData("Arm Mode Choose", m_armModeChooser);
      SmartDashboard.putData("Arm PID Testing", new RunArmFromDashboard(m_armModeChooser.getSelected(), m_arm));
      SmartDashboard.putData("Run Upper Arm PID", new RunCommand(m_arm::setUpperJointFromDashboardPos, m_arm));
    }                                  
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
