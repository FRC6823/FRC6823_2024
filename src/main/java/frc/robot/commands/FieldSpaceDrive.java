package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

//import edu.wpi.first.wpilibj2.command.Subsystem;
//import java.util.HashSet;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoystickHandler;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.MathUtil;

public class FieldSpaceDrive extends Command {
    // Declare subsystem, Joystick Handler, pigeon2
    private SwerveDriveSubsystem swerveDrive;
    private JoystickHandler joystickHandler;
    private Pigeon2Handler pigeon2Handler;
    private SimpleWidget speedRateWidget;
    private SimpleWidget turnRateWidget;
    private boolean drive;
    private PIDController yawPid;
    private int counter;
    private boolean slowMode;
    private boolean overrideYaw;

    public FieldSpaceDrive(SwerveDriveSubsystem subsystem,
            JoystickHandler joystickHandler, Pigeon2Handler pigeon2Handler) {
        // Instantiate subsystem, Joystick Handler, pigeon2
        this.swerveDrive = subsystem;
        this.joystickHandler = joystickHandler;
        this.pigeon2Handler = pigeon2Handler;
        this.speedRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Speed Rate", 0.5)
                .withWidget(BuiltInWidgets.kNumberSlider);
        this.turnRateWidget = Shuffleboard.getTab("Preferences").addPersistent("Turn Rate", 0.5)
                .withWidget(BuiltInWidgets.kNumberSlider);
        addRequirements(swerveDrive);
        yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
        yawPid.setSetpoint(pigeon2Handler.getYaw());
        yawPid.enableContinuousInput(0, 360);
        drive = true;
        counter = 0;
        slowMode = false;
        overrideYaw = false;
    }

    @Override
    public void execute() {
        pigeon2Handler.printEverything();
        joystickHandler.updateDeadZone();

        // Set speed and turn rates for full throttle and not full throttle
        double speedRate = speedRateWidget.getEntry().getDouble(1);
        double turnRate = turnRateWidget.getEntry().getDouble(1);
        double modeMultiplier = 1;

        if (slowMode) {
            modeMultiplier = 0.4;
        }

        // if (joystickHandler.isFullThrottle()) {
        // speedRate = 1;
        // turnRate = .6;
        // }

        // Set xval, yval, spinval to the scaled values from the joystick, bounded on
        // [-1, 1]
        double xval = joystickHandler.getAxis1() * -speedRate * 5 * modeMultiplier;
        double yval = joystickHandler.getAxis0() * -speedRate * 5 * modeMultiplier;
        double spinval = joystickHandler.getAxis5() * -turnRate * 5 * modeMultiplier;

        if (overrideYaw) {
            if (pigeon2Handler.getYaw() >= 90 && pigeon2Handler.getYaw() <= 270) {
                yawPid.setSetpoint(180);
            }
            else {
                yawPid.setSetpoint(0);
            }
            spinval = MathUtil.clipToRange(yawPid.calculate(pigeon2Handler.getYaw()), Math.PI);
        }
        
        else {
            if (joystickHandler.getRawAxis5() == 0 && (xval != 0 || yval != 0)) {
                if (counter == 0) {
                    spinval = yawPid.calculate(pigeon2Handler.getYaw());
                } else {
                    yawPid.setSetpoint(pigeon2Handler.getYaw());
                    counter--;
                }
            } else {
                yawPid.setSetpoint(pigeon2Handler.getYaw());
                counter = 20;
            }
        }
        // mapping field space to robot space
        // double txval = getTransX(xval, yval, robotAngle);
        // double tyval = getTransY(xval, yval, robotAngle);
        if (drive) {
            swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xval, yval, spinval, getRobotAngle()));
        }
    }

    public Rotation2d getRobotAngle() {
        return pigeon2Handler.getAngleDeg();
    }

    public void zero() { // Zeroes direction
        pigeon2Handler.zeroYaw();
    }

    public void drive(boolean drive) {
        this.drive = drive;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public void toggleOverrideYaw(){
        overrideYaw = !overrideYaw;
    }
}