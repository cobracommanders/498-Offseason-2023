package org.team498.C2023;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.team498.C2023.subsystems.drivetrain.Drivetrain;
import org.team498.C2023.subsystems.drivetrain.Gyro;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.util.PoseUtil;

import java.util.List;

public class Robot extends LoggedRobot {
    public static final double DEFAULT_PERIOD = 0.02;
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static Alliance alliance = Alliance.Invalid;
    public static final Controls controls = new Controls();

    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final SendableChooser<Auto> autoChooser = new SendableChooser<>();
    private Auto autoToRun;

    private boolean matchStarted = false;

    private final List<Auto> autoOptions = List.of(
    );

    @Override
    public void robotInit() {
        if (isReal()) Constants.mode = Constants.Mode.REAL;

        drivetrain.setYaw(0);

        FieldPositions.displayAll();

        autoChooser.setDefaultOption("Score", null);

        autoOptions.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        PathLib.eighthNodeToChargeStation.getClass();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (alliance == Alliance.Invalid) {
            alliance = DriverStation.getAlliance();
            // This reverses the coordinates/direction of the drive commands on the red alliance
            coordinateFlip = alliance == Alliance.Blue
                             ? 1
                             : -1;
            // Add 180 degrees to all teleop rotation setpoints while on the red alliance
            rotationOffset = alliance == Alliance.Blue
                             ? 0
                             : 180;
        }
    }

    @Override
    public void disabledPeriodic() {
        alliance = DriverStation.getAlliance();
        // This reverses the coordinates/direction of the drive commands on the red alliance
        coordinateFlip = alliance == Alliance.Blue
                         ? 1
                         : -1;
        // Add 180 degrees to all teleop rotation setpoints while on the red alliance
        rotationOffset = alliance == Alliance.Blue
                         ? 0
                         : 180;


        if (!matchStarted) {
            autoToRun = autoChooser.getSelected();
        }

        Drivetrain.getInstance().stop();
    }

    @Override
    public void teleopInit() {
        matchStarted = true;
    }

    @Override
    public void teleopPeriodic() {

    }


    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void autonomousInit() {
        matchStarted = true;

        if (autoToRun == null)
            autoToRun = null;

        autoToRun = autoChooser.getSelected();

        if (alliance == Alliance.Blue) {
            Drivetrain.getInstance().setYaw(autoToRun.getInitialPose().getRotation().getDegrees());
            Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        } else {
            Drivetrain.getInstance().setYaw(PoseUtil.flip(autoToRun.getInitialPose()).getRotation().getDegrees());
            Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        }

        autoToRun.getCommand().schedule();

        CommandScheduler.getInstance().run();

        if (alliance == Alliance.Blue) {
            Drivetrain.getInstance().setYaw(autoToRun.getInitialPose().getRotation().getDegrees());
            Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        } else {
            Drivetrain.getInstance().setYaw(PoseUtil.flip(autoToRun.getInitialPose()).getRotation().getDegrees());
            Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}