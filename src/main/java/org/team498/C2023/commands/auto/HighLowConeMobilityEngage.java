package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.drivetrain.chargestation.BangBangBalance;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.C2023.commands.robot.GroundIntake;
import org.team498.C2023.commands.robot.PrepareToScore;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.commands.robot.VerifyScoreLocation;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HighLowConeMobilityEngage implements Auto {
    private final Gyro gyro = Gyro.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CONE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                new PrepareToScore(),
                new VerifyScoreLocation(),
                new SetManipulatorToNextState(),
                new WaitCommand(.1),
                new InstantCommand(() -> drivetrain.setAngleGoal(180 - Robot.rotationOffset)),
                new ParallelCommandGroup(
                        new PathPlannerFollower(PathLib.sixthNodeToThirdCubeToChargeStation),
                        new SequentialCommandGroup(
                                new ReturnToIdle(),
                                new WaitCommand(3),
                                new SetRobotState(State.INTAKE),
                                new GroundIntake(),
                                new WaitCommand(3),
                                new ReturnToIdle()
                        )
                ),
                new ParallelCommandGroup(
                        new BangBangBalance(),  
                        new SequentialCommandGroup(
                                new SetRobotState(State.OUTTAKE),
                                new GroundIntake()
                        )
                )
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.sixthNodeToThirdCubeToChargeStation.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }

}
