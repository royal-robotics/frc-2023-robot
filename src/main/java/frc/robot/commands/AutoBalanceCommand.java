package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;
import frc.robot.Constants.Auto;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Timer;

public class AutoBalanceCommand extends CommandBase{
    private Swerve s_Swerve;
    private ProfiledPIDController s_yawController;
    private PIDController s_rollController; 
    private Pigeon2 s_gyro;
    private double s_roll;
    private double s_rollSpeed;
    private Timer timer;
    // private short[] s_accelerometer;
    
    

    public AutoBalanceCommand(Swerve swerve){
        s_Swerve = swerve; 
        addRequirements(s_Swerve);

        s_gyro = swerve.gyro;
    
        s_yawController = new ProfiledPIDController(1, 0, 0, Constants.Auto.kThetaControllerConstraints);
        s_yawController.enableContinuousInput(0, Math.PI);
        
        s_rollController = new PIDController(0.03, 0, 0);
        s_roll = s_gyro.getPitch();
        s_rollSpeed = 0;
        timer = new Timer();
        // s_accelerometer = new short[3];

        // ShuffleboardTab autoBalanceTest = Shuffleboard.getTab("Auto Balancing");
        //autoBalanceTest.addNumber("roll", () -> s_roll).withPosition(0, 0); 
        //autoBalanceTest.addNumber("rollspeed", () -> s_rollSpeed).withPosition(0, 1);

    }

    @Override
    public void initialize(){
       s_yawController.setGoal(0);
       s_rollController.setSetpoint(0);
       s_roll = s_gyro.getRoll();
       timer.start();
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

        //Rotation2d yaw = s_Swerve.getYaw();
        if (timer.get() % 1 <= 0.5) { 
            s_roll = s_gyro.getRoll();   // TODO: degrees or radians?

            /* 
            double yawSpeed = s_yawController.calculate(yaw.getRadians());
            if (s_yawController.atGoal()) {
                yawSpeed = 0;
            }
            */
            s_rollSpeed = s_rollController.calculate(s_roll);
        
            if(s_rollController.atSetpoint()) {
                s_rollSpeed = 0;
            }

            Translation2d translation = rollToTranslation(s_rollSpeed);
            s_Swerve.drive(translation, 0, false, false);    // TODO: are speeds robot relative?
        } else {
            s_Swerve.setStableModuleStates();
        }
    }

    @Override
    public void end(boolean interrupted){
        s_yawController.setGoal(0);
        s_rollController.setSetpoint(0);
        s_Swerve.setStableModuleStates(); // TODO: are speeds robot relative?
    }

    @Override
    public boolean isFinished(){
       // return s_yawController.atGoal() && s_pitchController.atSetpoint(); 
       return false; 
    }

    public Translation2d rollToTranslation(double rollError){
        // forward tilt = angle > 0 = drive backwards
        // backward tilt = angle < 0 = drive forward
        return new Translation2d(-rollError, 0);
    }
}
