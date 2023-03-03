package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoUp extends SequentialCommandGroup{
    public AutoUp(Arm arm, Intake intake) {
        //if (arm.getEncoder() < Constants.encoderAndSetPointLimit) {
            //if (intake.getBottomSolenoidValue() == Constants.intakeRetract ||
                    //intake.getTopSolenoidValue() == Constants.intakeRetract) {
                this.addCommands(new AutoExtendIntake(arm, intake, 0, 0.5));
            //}
            //if (arm.getSolenoidGrip() == Constants.gripOpen) {
                this.addCommands(new AutoGripClose(arm, 0.5));
            //}
            //if (arm.getSolenoidAngle() == Constants.armDown) {
                this.addCommands(new AutoGripUp(arm, intake, 1.0));
            //}
            this.addCommands(new AutoRetractIntake(arm, intake, 0.5));
        //}
    }
}
