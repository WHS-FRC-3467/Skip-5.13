// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.ClawDefault;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.auto.OneConeFar;
import frc.robot.auto.OneConeWithCharge;
import frc.robot.auto.TestAuto;
import frc.robot.subsystems.arm.ArmDefault;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TeleopSwerve;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.util.GamePiece;
import frc.robot.util.GamePiece.GamePieceType;

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

  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClawSubsytem m_claw = new ClawSubsytem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final LEDSubsystem m_led = new LEDSubsystem();

  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GamePiece.setGamePiece(GamePieceType.Cone);
    
    new Pneumatics();
    if(Constants.tuningMode){
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    else{
      DriverStation.silenceJoystickConnectionWarning(false);
    }
    // Configure the trigger bindings
    
    m_autoChooser.addOption("Test Auto", new TestAuto(m_drive));
    m_autoChooser.addOption("OneConeFar", new OneConeFar(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("OneConeWithCharge", new OneConeWithCharge(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("No Auto", null);
    SmartDashboard.putData("Auto", m_autoChooser);
    
    //new Pneumatics();
    configureBindings();
    m_drive.setDefaultCommand(new TeleopSwerve(m_drive, 
                                              () -> -m_driverController.getLeftY(), 
                                              () -> -m_driverController.getLeftX(), 
                                              () -> -m_driverController.getRightX(), 
                                              m_driverController.leftStick(),
                                              m_driverController.leftBumper(),
                                              m_driverController.rightBumper(),
                                              m_driverController.y(),
                                              m_driverController.b(),
                                              m_driverController.a(),
                                              m_driverController.x()));

    m_arm.setDefaultCommand(new ArmDefault(m_arm,
                                          m_operatorController.leftBumper(),
                                          () -> m_operatorController.getLeftY(), 
                                          () -> m_operatorController.getRightY()));

    m_claw.setDefaultCommand(new ClawDefault(m_claw, ()-> m_operatorController.getLeftTriggerAxis(), 
                                            () -> m_operatorController.getRightTriggerAxis()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGesnericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Driver Controls
    m_driverController.povUp().onTrue(new InstantCommand(m_drive::zeroGyro, m_drive));
    
    m_driverController.start().whileTrue(Commands.run(m_drive::AutoBalance, m_drive).andThen(m_drive::stopDrive, m_drive));

    m_driverController.rightTrigger().onTrue(Commands.runOnce(() -> m_arm.updateAllSetpoints(ArmSetpoints.MID_NODE_PLACED)).andThen(new WaitCommand(0.7)).andThen(m_arm::actuateClawOut));

    m_driverController.leftTrigger().onTrue(Commands.runOnce(() -> m_arm.updateAllSetpoints(ArmSetpoints.TOP_NODE_PLACED)).andThen(new WaitCommand(0.9)).andThen(m_arm::actuateClawOut));

    //Opperator Controls
    //Set game Piece type 
    m_operatorController.start().onTrue(Commands.runOnce(() -> GamePiece.toggleGamePiece()));
    m_operatorController.back().onTrue(Commands.runOnce(() -> GamePiece.toggleGamePiece()));

    //Set arm positions
    m_operatorController.rightBumper().onTrue(new RetractToStowed(m_arm));

    m_operatorController.a().onTrue(Commands.runOnce( () -> m_arm.updateAllSetpoints(ArmSetpoints.FLOOR)));

    m_operatorController.b().onTrue(new GoToPositionWithIntermediate(m_arm, ArmSetpoints.MID_NODE));

    m_operatorController.y().onTrue(new GoToPositionWithIntermediate(m_arm, ArmSetpoints.TOP_NODE));

    m_operatorController.x().onTrue(Commands.runOnce( () -> m_arm.updateAllSetpoints(ArmSetpoints.SUBSTATION)));

  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
