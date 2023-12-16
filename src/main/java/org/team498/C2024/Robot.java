package org.team498.C2024;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team498.C2024.commands.auto.TestAuto;
import org.team498.C2024.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.util.PoseUtil;
import java.util.List;
import java.util.Optional;


public class Robot extends TimedRobot{
    public static final double DEFAULT_PERIOD = 0.02;
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static Optional<Alliance> alliance = Optional.empty();
    public static final Controls controls = new Controls();

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();
    private final Blinkin blinkin = Blinkin.getInstance();
    //private final RobotState robotState = RobotState.getInstance();

    private final SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
    private Auto autoToRun;

    //private final Logger logger = Logger.getInstance();

    private boolean matchStarted = false;

    private final List<Auto> autoOptions = List.of(
           
                                                  );

    @Override
    public void robotInit() {
    //        new PowerDistribution(1, PowerDistribution.ModuleType.kRev).close(); // Enables power distribution logging
        drivetrain.setYaw(0);
       // FieldPositions.displayAll();
        autoChooser.setDefaultOption("Score", new TestAuto());
        autoOptions.forEach(auto -> autoChooser.addOption(auto.getName(), auto));
        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();
        PathLib.choreoTest.getClass();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (alliance.isEmpty()) {
            alliance = DriverStation.getAlliance();
            // This reverses the coordinates/direction of the drive commands on the red alliance
            coordinateFlip = alliance.get() == Alliance.Blue
                             ? 1
                             : -1;
            // Add 180 degrees to all teleop rotation setpoints while on the red alliance
            rotationOffset = alliance.get() == Alliance.Blue
                             ? 0
                             : 180;
        }
    }

    @Override
    public void disabledPeriodic() {
        alliance = DriverStation.getAlliance();
        // This reverses the coordinates/direction of the drive commands on the red alliance
        coordinateFlip = alliance.get() == Alliance.Blue
                         ? 1
                         : -1;
        // Add 180 degrees to all teleop rotation setpoints while on the red alliance
        rotationOffset = alliance.get() == Alliance.Blue
                         ? 0
                         : 180;


        if (!matchStarted) {
            autoToRun = autoChooser.getSelected();
            if (autoToRun != null) {
               // robotState.setState(autoToRun.getInitialState());
            }
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
            autoToRun = new TestAuto();

        //autoToRun = new HighHighCone();

        if (alliance.get() == Alliance.Blue) {
            Drivetrain.getInstance().setYaw(autoToRun.getInitialPose().getRotation().getDegrees());
            Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        } else {
            Drivetrain.getInstance().setYaw(PoseUtil.flip(autoToRun.getInitialPose()).getRotation().getDegrees());
            Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        }

        autoToRun.getCommand().schedule();

        CommandScheduler.getInstance().run();

        if (alliance.get() == Alliance.Blue) {
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

    @Override
    public void startCompetition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'startCompetition'");
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'endCompetition'");
    }
}