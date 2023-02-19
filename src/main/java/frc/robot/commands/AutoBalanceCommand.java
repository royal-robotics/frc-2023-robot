package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;
import frc.robot.Constants.Auto;
import frc.robot.subsystems.Swerve;


public class AutoBalanceCommand extends CommandBase{
    private Swerve s_Swerve;
    private ProfiledPIDController s_yawController;
    private PIDController s_pitchController; 
    private Pigeon2 s_gyro;
    private double s_pitch;
    // private short[] s_accelerometer;
    
    

    public AutoBalanceCommand(Swerve swerve, Pigeon2 gyro){
        s_Swerve = swerve; 
        addRequirements(s_Swerve);

        s_gyro = gyro;
    
        s_yawController = new ProfiledPIDController(1, 0, 0, Constants.Auto.kThetaControllerConstraints);
        s_yawController.enableContinuousInput(0, Math.PI);
        
        s_pitchController = new PIDController(0.1, 0, 0);
        s_pitch = s_gyro.getPitch();
        // s_accelerometer = new short[3];
    }

    @Override
    public void initialize(){
       s_yawController.setGoal(0);
       s_pitchController.setSetpoint(0);
       s_pitch = s_gyro.getPitch();
    }

    @Override
    public void execute(){
        /*
        // TODO: can use accelerometer + gyro value complementary filter if gyro alone isn't sufficient
        s_gyro.getBiasedAccelerometer(this.s_accelerometer);
        double accPitch = Math.atan2(s_accelerometer[1], s_accelerometer[2]);
        double pitchAngle = 0.9934 * (s_pitch + s_gyro.getPitch()) + 0.0066 *(accPitch);

        double pitchSpeed = (s_pitchController.atSetpoint()) ? 0 : s_pitchController.calculate(pitchAngle);
        */

        Rotation2d yaw = s_Swerve.getYaw();
        s_pitch = s_gyro.getPitch();   // TODO: degrees or radians?


        double yawSpeed = s_yawController.calculate(yaw.getRadians());
        if (s_yawController.atGoal()) {
            yawSpeed = 0;
        }

        double pitchSpeed = s_pitchController.calculate(s_pitch);
        if(s_pitchController.atSetpoint()) {
            pitchSpeed = 0;
        }

        Translation2d translation = pitchToTranslation(pitchSpeed);
        s_Swerve.drive(translation, yawSpeed, false, false);    // TODO: are speeds robot relative?
    }

    @Override
    public void end(boolean interrupted){
        s_yawController.setGoal(0);
        s_pitchController.setSetpoint(0);
        s_Swerve.drive(new Translation2d(), 0, false, false); // TODO: are speeds robot relative?
    }

    @Override
    public boolean isFinished(){
        return s_yawController.atGoal() && s_pitchController.atSetpoint(); 
    }

    public Translation2d pitchToTranslation(double pitchError){
        // forward tilt = angle > 0 = drive backwards
        // backward tilt = angle < 0 = drive forward
        return new Translation2d(-pitchError, 0);
        
    }
}
