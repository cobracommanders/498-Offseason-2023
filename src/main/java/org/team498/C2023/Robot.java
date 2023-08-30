package org.team498.C2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team498.C2023.commands.auto.*;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.elevator.Elevator;
import org.team498.C2023.subsystems.elevatorwrist.ElevatorWrist;
import org.team498.C2023.subsystems.intakerollers.IntakeRoller;
import org.team498.C2023.subsystems.intakewrist.IntakeWrist;
import org.team498.C2023.subsystems.manipulator.Manipulator;
import org.team498.lib.SystemsCheck;
import org.team498.lib.SystemsCheck.TestableObject;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.drivers.Blinkin.BlinkinColor;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.util.PoseUtil;
import org.team498.lib.util.RotationUtil;

import java.util.List;

import static org.team498.C2023.Ports.Accessories.SETUP_SWITCH;


public class Robot extends LoggedRobot {
    public static final double DEFAULT_PERIOD = 0.02;
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static Alliance alliance = Alliance.Invalid;
    public static final Controls controls = new Controls();

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();
    private final Blinkin blinkin = Blinkin.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<Auto>("AutoChooser");
    private Auto autoToRun;

    private final Logger logger = Logger.getInstance();

    private boolean matchStarted = false;

    public static final Mechanism2d mechanism2d = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(65));
    public static final MechanismRoot2d root = mechanism2d.getRoot("Root", Units.inchesToMeters(1), Units.inchesToMeters(1));
    public static final MechanismLigament2d base = root.append(new MechanismLigament2d("Drivetrain", Units.inchesToMeters(28), 0));

    public static final MechanismLigament2d elevatorBase = Robot.root.append(new MechanismLigament2d("Elevator Base", Units.inchesToMeters(3), 90));
    public static final MechanismLigament2d elevatorBase2 = elevatorBase.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(4), -30));
    public static final MechanismLigament2d elevatorMechanism = elevatorBase2.append(new MechanismLigament2d("Elevator", 0, 0));
    public static final MechanismLigament2d elevatorWristMechanism = elevatorMechanism.append(new MechanismLigament2d("Elevator Wrist", Units.inchesToMeters(15), 90));
    public static final MechanismLigament2d intakeWristMechanism = Robot.base.append(new MechanismLigament2d("Intake Wrist", Units.inchesToMeters(18), 0));

    private final List<Auto> autoOptions = List.of(
            new MobilityEngageCubeHigh(),
            new MobilityEngageConeMid(),
            new JustAutoShot(),
            new JustScore(),
            new CubeEngage(),
            new ConeTaxi(),
            new CubeTaxi(),
            new ConeTaxiBump(),
            new CubeTaxiBump(),
            new TwoPlusOneBump(),
            new HighMidCubeEngage(),
            new HighMidCubeEngageBump(),
            new HighMidCube(),
            new HighMidCubeBump(),
            new HighHighCubeEngage(),
            new HighHighCubeEngageBump(),
            new HighHighCube(),
            new HighHighCubeBump(),
            new ThreeBump(),
            new ThreeBumpEngage(),
            new TestAuto(),
            new HighHighCone()
            //new WaitCommand(10)
                                                  );

    private final DigitalInput setupSwitch = new DigitalInput(SETUP_SWITCH);

    public static final SystemsCheck fullCheck = new SystemsCheck("Full", 
        Commands.sequence(new ReturnToIdle(), Commands.runOnce(() -> Drivetrain.getInstance().X())),
        new TestableObject("Elevator/Encoder", () -> {}, () -> Elevator.getInstance().checkEncoderConnection(), 1),
        new TestableObject("ElevatorWrist/Encoder", () -> {}, () -> ElevatorWrist.getInstance().checkEncoderConnection(), 1),
        new TestableObject("IntakeWrist/Encoder", () -> {}, () -> IntakeWrist.getInstance().checkEncoderConnection(), 1),

        new TestableObject("IntakeWrist/Motors", () -> IntakeWrist.getInstance().setState(State.IntakeWrist.TEMPORARY_IDLE), () -> IntakeWrist.getInstance().atSetpoint(), 2),
        new TestableObject("ElevatorWrist/Motor", () -> ElevatorWrist.getInstance().setState(State.ElevatorWrist.TRAVEL), () -> ElevatorWrist.getInstance().atSetpoint(), 2),
        new TestableObject("Elevator/Motors", () -> Elevator.getInstance().setState(State.Elevator.TOP_CUBE), () -> Elevator.getInstance().atSetpoint(), 2),
        new TestableObject("Manipulator/Motor", () -> Manipulator.getInstance().setState(State.Manipulator.INTAKE_CUBE), () -> Manipulator.getInstance().getCurrentDraw() > 0.2, 2),
        new TestableObject("IntakeRoller/Motors", () -> IntakeRoller.getInstance().setState(State.IntakeRollers.INTAKE), () -> {
            var amps = IntakeRoller.getInstance().getCurrentDraws();
            return amps[0] + amps[1] + amps[2] > 0.6;
        }, 2)//,
        // new TestableObject("Drivetrain/Motors", () -> Drivetrain.getInstance().drive(1, 0, 0, true), () ->
            // Math.abs(Drivetrain.getInstance().getCurrentSpeeds().vxMetersPerSecond - 1) < 0.05,
            // 2)
    );


    @Override
    public void robotInit() {
        if (isReal()) Constants.mode = Constants.Mode.REAL;

        //logger.recordMetadata("ProjectName", "C2023");
        //logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        //logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        //logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        //logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        //logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        //logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        if (isReal()) {
            //            logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to the rio directly
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev).close(); // Enables power distribution logging
        } else {
            switch (Constants.mode) {
                case SIM -> {
                    logger.addDataReceiver(new WPILOGWriter("/"));
                    logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                }
                case REPLAY -> {
                    setUseTiming(false); // Set as false to run as fast as possible when replaying logs
                    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                    logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                    logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                }
            }
        }

        logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        drivetrain.setYaw(0);

        FieldPositions.displayAll();

        autoChooser.addDefaultOption("Score", new JustScore());

        autoOptions.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        PathLib.eighthNodeToChargeStation.getClass();

        Elevator.getInstance().setBrakeMode(false);
        ElevatorWrist.getInstance().setBrakeMode(false);

        new Trigger(() -> setupSwitch.get() && isDisabled())
                .toggleOnTrue(Commands.startEnd(() -> {
                                                    Elevator.getInstance().setBrakeMode(true);
                                                    ElevatorWrist.getInstance().setBrakeMode(true);
                                                },
                                                () -> {
                                                    Elevator.getInstance().setBrakeMode(false);
                                                    ElevatorWrist.getInstance().setBrakeMode(false);
                                                }
                                               ));
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

        //logger.recordOutput("Mechanism2d", mechanism2d);

        //logger.recordOutput("Targets/NextScoringPose", RobotPosition.getNextScoringNodePosition());
        //logger.recordOutput("Targets/FutureScoringPose", RobotPosition.getFutureScoringNodePosition());
        //logger.recordOutput("Targets/FutureRobotPose", RobotPosition.getFuturePose(20));
        //logger.recordOutput("Targets/Distance", RobotPosition.getFutureScoringNodeDistance());

        //SmartDashboard.putNumber("Pitch", Gyro.getInstance().getPitch());
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
            autoToRun = autoChooser.get();
            if (autoToRun != null) {
                robotState.setState(autoToRun.getInitialState());
                Elevator.getInstance().setState(autoToRun.getInitialState().elevator);
                ElevatorWrist.getInstance().setState(autoToRun.getInitialState().elevatorWrist);
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
        if (RobotPosition.isReadyToScore(Drivetrain.getInstance().getPose(), 0)){
            blinkin.setColor(BlinkinColor.SOLID_LIME);
            controls.driver.rumble(0.5);
        } else if (robotState.inShootDriveMode() && RobotPosition.inCommunity()) {
            blinkin.setColor(BlinkinColor.LIGHT_CHASE_RED);
        } else {
            if (robotState.inConeMode() && Manipulator.getInstance().isStalling()) {
                blinkin.setColor(BlinkinColor.SOLID_BLUE);
            } else {
                blinkin.setColor(RobotState.getInstance().inConeMode()
                                 ? BlinkinColor.SOLID_YELLOW//RAINBOW_PALETTE
                                 : BlinkinColor.SOLID_VIOLET);
            }
            controls.driver.rumble(0);
        }
    }


    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void autonomousInit() {
        matchStarted = true;

        if (autoToRun == null)
            autoToRun = new HighHighCone();

        autoToRun = new HighHighCone();

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