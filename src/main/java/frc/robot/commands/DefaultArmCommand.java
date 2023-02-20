package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase{
    private Arm s_Arm;
    private DoubleSupplier s_Speed;
    private BooleanSupplier s_Grip;
    private BooleanSupplier s_Angle;

    private DoubleSolenoid.Value gripValue;
    private DoubleSolenoid.Value angleValue;
    private boolean gripPressed;
    private boolean anglePressed;

    public DefaultArmCommand(Arm arm, DoubleSupplier speed, BooleanSupplier grip, BooleanSupplier angle){
        s_Arm = arm;
        addRequirements(s_Arm);

        s_Speed = speed;
        s_Grip = grip;
        s_Angle = angle;
    }

    @Override
    public void initialize(){
        s_Arm.setMotorSpeed(0);
        s_Arm.setSolenoidGrip(DoubleSolenoid.Value.kReverse);
        s_Arm.setSolenoidAngle(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void execute(){
        // Toggle grip
        if (s_Grip.getAsBoolean()) {
            if (!gripPressed) {
                gripPressed = true;
                if (gripValue == DoubleSolenoid.Value.kReverse) {
                    gripValue = DoubleSolenoid.Value.kForward;
                } else {
                    gripValue = DoubleSolenoid.Value.kReverse;
                }
            }
        } else {
            gripPressed = false;
        }
        
        // Toggle angle
        if (s_Angle.getAsBoolean()) {
            if (!anglePressed) {
                anglePressed = true;
                if (angleValue == DoubleSolenoid.Value.kReverse) {
                    angleValue = DoubleSolenoid.Value.kForward;
                } else {
                    angleValue = DoubleSolenoid.Value.kReverse;
                }
            }
        } else {
            anglePressed = false;
        }

        s_Arm.setMotorSpeed(s_Speed.getAsDouble());
        s_Arm.setSolenoidGrip(gripValue);
        s_Arm.setSolenoidAngle(angleValue);
    }

    @Override
    public void end(boolean interrupted){
        s_Arm.setMotorSpeed(0);
        s_Arm.setSolenoidGrip(DoubleSolenoid.Value.kReverse);
        s_Arm.setSolenoidAngle(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
