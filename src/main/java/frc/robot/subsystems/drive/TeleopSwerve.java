package frc.robot.subsystems.drive;

import frc.robot.Constants.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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

    private double rotationVal, xVal, yVal;
    double m_angle = 0d;
    private PIDController m_thetaController;
    ;
  
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
                        BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy){
        m_Swerve = swerve;
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
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(SwerveConstants.GAINS_ANGLE_SNAP.kP, SwerveConstants.GAINS_ANGLE_SNAP.kI, SwerveConstants.GAINS_ANGLE_SNAP.kD);
        
        m_thetaController.enableContinuousInput(-180, 180);
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("gyro Yaw", m_Swerve.getYaw().getDegrees());
        /* Get Values, Deadband*/
        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();
        xVal = MathUtil.applyDeadband(xSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        yVal = MathUtil.applyDeadband(ySup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        SmartDashboard.putBoolean("rotate with button", rotateWithButton);

        if(rotateWithButton){
            if(m_0.getAsBoolean()){
                m_thetaController.setSetpoint(0.0);

            }
            else if(m_90.getAsBoolean()){
                m_thetaController.setSetpoint(90.0);
            }
            else if(m_180.getAsBoolean()){
                m_thetaController.setSetpoint(180.0);
            }
            else if(m_270.getAsBoolean()){
                m_thetaController.setSetpoint(-90.0);
            }
            rotationVal = m_thetaController.calculate(-(MathUtil.inputModulus(m_Swerve.getYaw().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
            SmartDashboard.putNumber("RotationVal", rotationVal);
            SmartDashboard.putNumber("Theta Controller setpoint", m_thetaController.getSetpoint());
            SmartDashboard.putNumber("Yaw", m_Swerve.getYaw().getDegrees() % 180.0);
        }
        else if (!rotateWithButton){
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        }


        if(m_quarterSpeed.getAsBoolean()){
            xVal = xVal*0.25;
            yVal =yVal*0.25;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.25;
            }
        }
        else if(m_halfSpeed.getAsBoolean()){
            xVal = xVal*0.5;
            yVal =yVal*0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        

        m_Swerve.drive(
            new Translation2d(xVal, yVal).times(SwerveConstants.MAX_SPEED), 
            rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
            true,
            false,
            true);
    }
}