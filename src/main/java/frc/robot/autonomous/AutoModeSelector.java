package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonomous.autoModes.*;
import java.util.function.Supplier;

public class AutoModeSelector {
    public final SendableChooser<Supplier<SequentialCommandGroup>> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<Supplier<SequentialCommandGroup>>();
        _chooser.addOption("autoMode", () -> new AutoMode(robotContainer));
        _chooser.addOption("autoMode2", () -> new AutoMode2(robotContainer));
        _chooser.addOption("middlePath", () -> new MiddlePath(robotContainer));
        _chooser.addOption("push", () -> new Push(robotContainer));
        _chooser.addOption("top", () -> new Top(robotContainer));
        _chooser.addOption("bottom", () -> new Bottom(robotContainer));
    }

    public SequentialCommandGroup getAutoMode() {
        return _chooser.getSelected().get();
    }
}
