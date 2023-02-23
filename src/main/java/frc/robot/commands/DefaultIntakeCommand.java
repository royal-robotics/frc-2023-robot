package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class DefaultIntakeCommand extends CommandBase{

    private Intake s_Intake;
    private DoubleSupplier s_Speed;
    private BooleanSupplier s_SolenoidBottom;
    private BooleanSupplier s_SolenoidTop;

    private DoubleSolenoid.Value bottomValue;
    private DoubleSolenoid.Value topValue;
    private boolean bottomPressed;
    private boolean topPressed;

    public DefaultIntakeCommand(Intake intake, DoubleSupplier speed, BooleanSupplier solenoidBottom, BooleanSupplier solenoidTop){
        s_Intake = intake;
        addRequirements(s_Intake);

        s_Speed = speed;
        s_SolenoidBottom = solenoidBottom;
        s_SolenoidTop = solenoidTop;

        bottomValue = DoubleSolenoid.Value.kReverse;
        topValue = DoubleSolenoid.Value.kReverse;
        bottomPressed = false;
        topPressed = false;
    }
    
    @Override
    public void initialize(){
        s_Intake.setMotorSpeed(0);
        s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){
        // Toggle bottom
        if (s_SolenoidBottom.getAsBoolean()) {
            if (!bottomPressed) {
                bottomPressed = true;
                if (bottomValue == DoubleSolenoid.Value.kReverse) {
                    bottomValue = DoubleSolenoid.Value.kForward;
                } else {
                    bottomValue = DoubleSolenoid.Value.kReverse;
                }
            }
        } else {
            bottomPressed = false;
        }
        
        // Toggle top
        if (s_SolenoidTop.getAsBoolean()) {
            if (!topPressed) {
                topPressed = true;
                if (topValue == DoubleSolenoid.Value.kReverse) {
                    topValue = DoubleSolenoid.Value.kForward;
                } else {
                    topValue = DoubleSolenoid.Value.kReverse;
                }
            }
        } else {
            topPressed = false;
        }

        s_Intake.setMotorSpeed(s_Speed.getAsDouble());
        s_Intake.setBottomSolenoidValue(bottomValue);
        s_Intake.setTopSolenoidValue(topValue);
    }

    @Override
    public void end(boolean interrupted){
        s_Intake.setMotorSpeed(0);
        s_Intake.setSolenoidValue(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
