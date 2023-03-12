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
    private BooleanSupplier disableXSup;
    private BooleanSupplier lockForwardSup;
    private BooleanSupplier lockBackwardSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier disableXSup, BooleanSupplier lockForwardSup, BooleanSupplier lockBackwardSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.disableXSup = disableXSup;
        this.lockForwardSup = lockForwardSup;
        this.lockBackwardSup = lockBackwardSup;
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

        if (lockForwardSup.getAsBoolean()) {
            rotationVal = s_Swerve.getYaw().getDegrees();
            while (rotationVal > 180) {
                rotationVal -= 360;
            }
            while (rotationVal < -180) {
                rotationVal += 360;
            }
            rotationVal *= -Constants.Drivebase.rotationValMultiplier;
            if (rotationVal > 0.6) {
                rotationVal = 0.6;
            } else if (rotationVal < -0.6) {
                rotationVal = -0.6;
            }
        } else if (lockBackwardSup.getAsBoolean()) {
            rotationVal = s_Swerve.getYaw().getDegrees() + 180;
            while (rotationVal > 180) {
                rotationVal -= 360;
            }
            while (rotationVal < -180) {
                rotationVal += 360;
            }
            rotationVal *= -Constants.Drivebase.rotationValMultiplier;
            if (rotationVal > 0.6) {
                rotationVal = 0.6;
            } else if (rotationVal < -0.6) {
                rotationVal = -0.6;
            }
        }
        /* Drive */

        if (DriverStation.isTeleop()) {
            if (translationVal == 0 && strafeVal == 0 && rotationVal == 0 && !disableXSup.getAsBoolean()) {
                if (s_Swerve.m_speedMultiplier == Constants.slowMode){
                    s_Swerve.drive(new Translation2d(0,0), 0, true, true);
                }else {
                    s_Swerve.setStableModuleStates();
                }
            } else {
                s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
                    rotationVal * SwerveConstants.maxAngularVelocity, 
                    true, 
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