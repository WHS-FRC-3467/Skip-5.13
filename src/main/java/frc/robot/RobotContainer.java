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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.claw.ClawDefault;
import frc.robot.subsystems.claw.ClawSubsytem;
import frc.robot.subsystems.cubeShooter.CubeShooterSubsystem;
import frc.robot.subsystems.cubeShooter.Shoot;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.auto.OneConeChargeWithCubePickup;
import frc.robot.auto.OneConeChargeWithMobility;
import frc.robot.auto.OneConeWithCharge;
import frc.robot.auto.OverBumpTwoPiece;
import frc.robot.auto.RealThreePiece;
import frc.robot.auto.ThreePiece;
import frc.robot.auto.ThreePieceTryThree;
import frc.robot.auto.TwoPiece;
import frc.robot.auto.TwoPieceWithCharge;
import frc.robot.subsystems.arm.ArmDefault;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.GoToMidNode;
import frc.robot.subsystems.arm.GoToPositionWithIntermediate;
import frc.robot.subsystems.arm.RetractToStowed;
import frc.robot.subsystems.arm.ScoreAndRetract;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TeleopSwerve;
import frc.robot.subsystems.led.LEDDefault;
import frc.robot.subsystems.led.LEDSubsystem;
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
  //private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final LEDSubsystem m_led = new LEDSubsystem();
  private final CubeShooterSubsystem m_shooter = new CubeShooterSubsystem();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  
  // private final PoseEstimatorSubsystem m_EstimatorSubsystem = new PoseEstimatorSubsystem(m_limelight, m_drive);
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
    
    m_autoChooser.addOption("Charge and no mobility", new OneConeWithCharge(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Charge and mobility", new OneConeChargeWithMobility(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Charge with cube pickup", new OneConeChargeWithCubePickup(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Two Game Piece", new TwoPiece(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Two piece with charge", new TwoPieceWithCharge(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Over bump two piece", new OverBumpTwoPiece(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Three Piece", new ThreePiece(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Real Three Piece", new RealThreePiece(m_drive, m_arm, m_claw));
    m_autoChooser.addOption("Cuby Three Piece", new ThreePieceTryThree(m_arm, m_drive, m_claw, m_shooter));
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
                                              m_driverController.rightStick(),
                                              m_driverController.y(),
                                              m_driverController.b(),
                                              m_driverController.a(),
                                              m_driverController.x(),
                                              m_arm));

    m_arm.setDefaultCommand(new ArmDefault(m_arm,
                                          m_operatorController.leftBumper(),
                                          () -> m_operatorController.getLeftY(), 
                                          () -> m_operatorController.getRightY()));

    m_claw.setDefaultCommand(new ClawDefault(m_claw, ()-> m_operatorController.getLeftTriggerAxis(), 
                                            () -> m_operatorController.getRightTriggerAxis()));

    m_led.setDefaultCommand(new LEDDefault(m_led, m_claw, m_shooter));

    m_shooter.setDefaultCommand(new Shoot(m_shooter, 
                                          ()-> m_driverController.getRightTriggerAxis()));

    SmartDashboard.putData("Reset Modules", new InstantCommand(m_drive::resetModulesToAbsolute));
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
    m_driverController.povUp().onTrue(Commands.runOnce(m_drive::reset, m_drive));

    m_driverController.start().whileTrue(Commands.run(m_drive::AutoBalance, m_drive).andThen(m_drive::stopDrive, m_drive));
    
    m_driverController.leftTrigger().onTrue(new ScoreAndRetract(m_arm, m_claw));

    m_driverController.rightBumper().onTrue(Commands.runOnce(() -> m_arm.updateAllSetpoints(ArmSetpoints.SUBSTATION)));
    

    //Opperator Controls
    //Set game Piece type 
    m_operatorController.back().onTrue (Commands.runOnce(() -> GamePiece.toggleGamePiece()));
    m_operatorController.start().onTrue (Commands.runOnce( () -> m_arm.updateAllSetpoints(ArmSetpoints.FLOOR_HOVER)));

    //Set arm positions
    m_operatorController.rightBumper().onTrue(new RetractToStowed(m_arm));

    m_operatorController.a().onTrue(Commands.runOnce( () -> m_arm.updateAllSetpoints(ArmSetpoints.FLOOR)));

    m_operatorController.b().onTrue(new GoToMidNode(m_arm));

    m_operatorController.y().onTrue(new GoToPositionWithIntermediate(m_arm, ArmSetpoints.TOP_NODE));

    m_operatorController.x().onTrue(Commands.runOnce(() -> m_arm.updateAllSetpoints(ArmSetpoints.SUBSTATION)));

    m_operatorController.povRight().whileTrue(Commands.run(()-> m_shooter.shoot(-0.4), m_shooter));
    m_operatorController.povLeft().whileTrue(Commands.run(()-> m_shooter.shoot(0.3), m_shooter));
    m_operatorController.povUp().whileTrue(Commands.run(()-> m_shooter.shoot(-0.9), m_shooter));
    m_operatorController.povDown().whileTrue(Commands.run(()-> m_shooter.shoot(-0.15), m_shooter));
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
