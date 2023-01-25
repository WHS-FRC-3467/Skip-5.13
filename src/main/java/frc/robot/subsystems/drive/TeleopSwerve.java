package frc.robot.subsystems.drive;

import frc.robot.Constants.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private DriveSubsystem s_Swerve;    
    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotationSup;
    //private BooleanSupplier robotCentricSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
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
    public TeleopSwerve(DriveSubsystem s_Swerve, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        //this.robotCentricSup = robotCentricSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double xVal = MathUtil.applyDeadband(xSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        double yVal = MathUtil.applyDeadband(ySup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);

        if(m_quarterSpeed.getAsBoolean()){
            xVal = xVal*0.25;
            yVal =yVal*0.25;
            rotationVal = rotationVal*0.25;
            s_Swerve.drive(
                new Translation2d(xVal, yVal).times(SwerveConstants.MAX_SPEED), 
                rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
                true,
                // !robotCentricSup.getAsBoolean(), 
                true);
        }
        else if(m_halfSpeed.getAsBoolean()){
            xVal = xVal*0.5;
            yVal =yVal*0.5;
            rotationVal = rotationVal*0.5;
            s_Swerve.drive(
                new Translation2d(xVal, yVal).times(SwerveConstants.MAX_SPEED), 
                rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
                true,
                // !robotCentricSup.getAsBoolean(), 
                true);

        }
        else{
            s_Swerve.drive(
                    new Translation2d(xVal, yVal).times(SwerveConstants.MAX_SPEED), 
                    rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
                    true,
                    // !robotCentricSup.getAsBoolean(), 
                    true
                );
            }
    }
}