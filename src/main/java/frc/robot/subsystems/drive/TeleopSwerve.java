package frc.robot.subsystems.drive;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.Setpoint.ArmState;
import frc.robot.util.GeomUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private DriveSubsystem m_Swerve;    
    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    private BooleanSupplier m_90, m_180, m_270, m_0;

    private double rotationVal;
    double m_angle = 0d;
    private PIDController m_thetaController;
    private SendableChooser<Double> m_speedChooser;
    ArmSubsystem m_arm;
  
    /**
     * 
     * @param s_Swerve
     * @param xSup
     * @param ySup
     * @param rotationSup
     * @param robotCentricSup
     * @param halfSpeed
     * @param quarterSpeed
     */
    public TeleopSwerve(DriveSubsystem swerve, DoubleSupplier xSup, DoubleSupplier ySup, 
                        DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, 
                        BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed, 
                        BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy, ArmSubsystem arm){
        m_Swerve = swerve;
        m_arm = arm;
        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        addRequirements(m_Swerve);
        m_speedChooser = new SendableChooser<Double>();
        m_speedChooser.addOption("100%", 1.0);
        m_speedChooser.addOption("97%", 0.97);
        m_speedChooser.setDefaultOption("95%", 0.95);
        m_speedChooser.addOption("92%", 0.92);
        m_speedChooser.addOption("90%", 0.9);
        m_speedChooser.addOption("85%", 0.85);
        m_speedChooser.addOption("80%", 0.8);
        m_speedChooser.addOption("70%", 0.7);
        m_speedChooser.addOption("60%", 0.6);
        SmartDashboard.putData("Speed Percent", m_speedChooser);
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(SwerveConstants.GAINS_ANGLE_SNAP.kP, SwerveConstants.GAINS_ANGLE_SNAP.kI, SwerveConstants.GAINS_ANGLE_SNAP.kD);
        
        m_thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        /* Get Values, Deadband*/
        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();
        double x = xSup.getAsDouble();
        double y = ySup.getAsDouble();    
                
        double linearMagnitude = Math.hypot(x, y);
        Rotation2d linearDirection = new Rotation2d(x, y);
        linearMagnitude = MathUtil.applyDeadband(linearMagnitude, SwerveConstants.DRIVE_DEADBAND);

        linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
        linearMagnitude *= m_speedChooser.getSelected();
        
        // SmartDashboard.putBoolean("rotate with button", rotateWithButton);
        
        if(rotateWithButton){
            if(m_0.getAsBoolean()){
                m_thetaController.setSetpoint(0.0);
            }
            else if(m_90.getAsBoolean()){
                m_thetaController.setSetpoint(-90.0);
            }
            else if(m_180.getAsBoolean()){
                m_thetaController.setSetpoint(180.0);
            }
            else if(m_270.getAsBoolean()){
                m_thetaController.setSetpoint(90.0);
            }
            rotationVal = m_thetaController.calculate((MathUtil.inputModulus(m_Swerve.getPose().getRotation().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -SwerveConstants.MAX_ANGULAR_VELOCITY * 0.075, SwerveConstants.MAX_ANGULAR_VELOCITY * 0.075);
            // SmartDashboard.putNumber("RotationVal", rotationVal);
            // SmartDashboard.putNumber("Theta Controller setpoint", m_thetaController.getSetpoint());
        }
        else if (!rotateWithButton){
            rotationVal = (MathUtil.applyDeadband(rotationSup.getAsDouble() * m_speedChooser.getSelected(), SwerveConstants.DRIVE_DEADBAND))*0.75;
            rotationVal = Math.copySign(rotationVal * rotationVal, rotationVal);
        }
        
        
        if(m_quarterSpeed.getAsBoolean()){
            linearMagnitude *= 0.25;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.25;
            }
        }
        else if(m_halfSpeed.getAsBoolean()){
            linearMagnitude *=0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        else if(m_arm.getSetpoint().state == ArmState.MID_NODE || 
                m_arm.getSetpoint().state == ArmState.TOP_NODE || 
                m_arm.getSetpoint().state == ArmState.MID_NODE_PLACED ||
                m_arm.getSetpoint().state == ArmState.TOP_NODE_PLACED ||
                m_arm.getSetpoint().state == ArmState.SUBSTATION ){
            linearMagnitude *= 0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        else{
            linearMagnitude *= 1.0;
            if(!rotateWithButton){
                rotationVal = rotationVal *1.0;
            } 
        }
        
        
        // Calcaulate new linear components
        Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();
        
        m_Swerve.drive(
            linearVelocity.times(SwerveConstants.MAX_SPEED), 
            rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY * 0.9, 
            true,
            false);
    }
}