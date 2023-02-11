package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.GridAlignCommand.*;
import frc.robot.commands.GridAlignRobotSpaceCommand.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final JoystickButton zeroGyro = new JoystickButton(driver, 11);
    private final JoystickButton robotCentric = new JoystickButton(driver, 10);
    private final JoystickButton alignAprilTagField = new JoystickButton(driver, 9);
    private final JoystickButton alignAprilTagRobot = new JoystickButton(driver, 8);

    /* Subsystems */
    
    public final Visions s_Visions = new Visions();
    public final Swerve s_Swerve = new Swerve(s_Visions);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        alignAprilTagField.onTrue(new GridAlignCommand(s_Swerve, GridAlignCommand.Align.CENTER));
        alignAprilTagRobot.onTrue(new GridAlignRobotSpaceCommand(s_Swerve,  GridAlignRobotSpaceCommand.Align.CENTER));
    }
}