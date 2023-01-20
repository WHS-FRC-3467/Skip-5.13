package frc.robot.subsystems.drive;

import frc.robot.Constants.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private DriveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    // private BooleanSupplier robotCentricSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    public TeleopSwerve(DriveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        // this.robotCentricSup = robotCentricSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwerveConstants.DRIVE_DEADBAND);

        if(m_quarterSpeed.getAsBoolean()){
            translationVal = translationVal*0.25;
            strafeVal =strafeVal*0.25;
            rotationVal = rotationVal*0.25;
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED), 
                rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
                false,
                // !robotCentricSup.getAsBoolean(), 
                true);
        }
        else if(m_halfSpeed.getAsBoolean()){
            translationVal = translationVal*0.5;
            strafeVal =strafeVal*0.5;
            rotationVal = rotationVal*0.5;
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED), 
                rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
                // !robotCentricSup.getAsBoolean(), 
                false,
                true);
        }
        else{
            s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED), 
            rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY, 
            // !robotCentricSup.getAsBoolean(), 
            false,
            true);
        }
    }
}