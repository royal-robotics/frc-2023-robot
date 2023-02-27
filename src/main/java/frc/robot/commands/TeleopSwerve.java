package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.lib.util.SwerveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (translationVal < 0) {
            translationVal *= -translationVal;
        } else {
            translationVal *= translationVal;
        }
        if (strafeVal < 0) {
            strafeVal *= -strafeVal;
        } else {
            strafeVal *= strafeVal;
        }
        if (rotationVal < 0) {
            rotationVal *= -rotationVal;
        } else {
            rotationVal *= rotationVal;
        }

        /* Drive */

        if (DriverStation.isTeleop()) {
            if (translationVal == 0 && strafeVal == 0 && rotationVal == 0) {
                s_Swerve.setStableModuleStates();
            } else {
                s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
                    rotationVal * SwerveConstants.maxAngularVelocity, 
                    !robotCentricSup.getAsBoolean(), 
                    true
                );
            }
        }

        // s_Swerve.drive(
        //     new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
        //     rotationVal * SwerveConstants.maxAngularVelocity, 
        //     !robotCentricSup.getAsBoolean(), 
        //     true
        // );
    }
}