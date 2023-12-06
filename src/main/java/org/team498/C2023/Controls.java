package org.team498.C2023;

import org.team498.C2023.Constants.OIConstants;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.drivetrain.DefenseDrive;
import org.team498.C2023.commands.drivetrain.HybridDrive;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.commands.drivetrain.TargetDrive;
import org.team498.C2023.commands.elevator.ManualElevator;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.ManualElevatorWrist;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.robot.*;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.elevatorwrist.ElevatorWrist;
import org.team498.C2023.subsystems.manipulator.Manipulator;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Xbox;
import org.team498.lib.wpilib.ChoiceCommand;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    private final RobotState robotState = RobotState.getInstance();

    public Controls() {
        driver.setLeftDeadzone(0.2);
        driver.setRightDeadzone(0.2);
        driver.setTriggerThreshold(0.2);
        operator.setLeftDeadzone(0.2);
        operator.setRightDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        Drivetrain.getInstance()
                  //.setDefaultCommand(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper()));
                  .setDefaultCommand(new HybridDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver::rawPOVAngle, driver.rightBumper()));
    }

    public void configureDriverCommands() {
        driver.leftTrigger()
              .onTrue(runOnce(() -> robotState.setState(State.INTAKE)).andThen(new GroundIntake()))
              .onFalse(new ReturnToIdle());
              //.whileTrue(new OffenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightAngle, driver.rightBumper()));
        driver.leftBumper()
              .onTrue(runOnce(() -> robotState.setState(State.OUTTAKE)).andThen(new GroundIntake()))
              .onFalse(new ReturnToIdle());
        driver.A().onTrue(runOnce(() -> Drivetrain.getInstance().setYaw(0)));
        driver.B().onTrue(new RealignCone());

        driver.rightStick().and(RobotPosition::inCommunity)
              .onTrue(new ShootWhileMoving())
              .onFalse(parallel(new ReturnToIdle(), runOnce(() -> Drivetrain.getInstance().getCurrentCommand().cancel())));
        driver.rightBumper()
        .whileTrue(new ConditionalCommand(new HybridDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver::rawPOVAngle, driver.rightBumper()),(new OffenseDrive(driver::leftYSquared, driver::leftXSquared, ()-> (0-Robot.rotationOffset), driver.rightBumper())), ()-> robotState.getState() != State.DOUBLE_SS));
        driver.rightTrigger()
              .whileTrue(either(
                      new TargetDrive(/*() -> 0*/driver::leftYSquared , driver::leftXSquared, driver.rightBumper(), RobotPosition::getFutureScoringNodePosition),
                      none(),
                      () -> RobotState.getInstance().inCubeMode()
                              && RobotState.getInstance().getNextScoringOption() != ScoringOption.SPIT
                              && RobotState.getInstance().inShootDriveMode()
                               ))
              .onTrue(new PrepareToScore())//.alongWith(new OffenseDrive(driver::leftYSquared, driver::leftXSquared, ()-> 180 - Robot.rotationOffset, driver.rightBumper())))
              .onFalse(new ChoiceCommand(() -> switch (robotState.getNextScoringOption()) {
                  case TOP, MID, LOW, AUTO_MID_CONE, AUTO_TOP_CONE -> sequence(new VerifyScoreLocation(),
                                            either(
                                                    waitSeconds(robotState.getNextScoringOption() == ScoringOption.LOW ? 0 : 0),
                                                    waitSeconds(0.1),
                                                    () -> RobotState.getInstance().inConeMode()
                                                  ),
                                            new Score()
                                           );
                  case SPIT -> new Spit();
              }));//.alongWith(new DefenseDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver.rightBumper())));

        driver.X().onTrue(new Spit());
        driver.Y().onTrue(Robot.fullCheck.test());

        driver.start().whileTrue(new FixCube()).onFalse(new ReturnToIdle());
    }

    public void configureOperatorCommands() {
        operator.Y().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.TOP)));
        operator.B().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.MID)));
        operator.A().onTrue(runOnce(() -> robotState.setNextScoringOption(RobotState.ScoringOption.LOW)));
        operator.X().toggleOnTrue(startEnd(() -> robotState.setShootDrive(true), () -> robotState.setShootDrive(false)));

        operator.rightBumper().onTrue(runOnce(() -> robotState.setCurrentGameMode(GameMode.CONE)));
        operator.leftBumper().onTrue(runOnce(() -> robotState.setCurrentGameMode(GameMode.CUBE)));

        operator.leftTrigger()
        .onTrue(new CollectFromSS()
        .deadlineWith(new OffenseDrive(driver::leftYSquared, driver::leftXSquared, ()-> (0), driver.rightBumper())))
        .onFalse(new ReturnToIdle());

        operator.rightTrigger()
                .toggleOnTrue(either(runOnce(() -> robotState.setState(State.SPIT_CUBE)),
                                     runOnce(() -> robotState.setState(State.IDLE_CUBE)),
                                     () -> robotState.getState() == State.IDLE_CUBE
                                    ).andThen(parallel(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState(),
                        new SetElevatorToNextState(),
                        new SetIntakeRollersToNextState()
                                                      )));

        operator.start().toggleOnTrue(new ManualElevator(() -> -operator.rightY()));
        operator.back().toggleOnTrue(new ManualElevatorWrist(() -> -operator.leftY()));

        operator.POV90().whileTrue(runOnce(() -> Manipulator.getInstance().setState(State.Manipulator.INTAKE_CONE)));
        operator.POVMinus90().whileTrue(runOnce(() -> Manipulator.getInstance().setState(State.Manipulator.MID_CONE)));

        operator.POV0().onTrue(runOnce(() -> ElevatorWrist.getInstance().incrementOffset(0.01)));
        operator.POV180().onTrue(runOnce(() -> ElevatorWrist.getInstance().incrementOffset(-0.01)));
    }
}