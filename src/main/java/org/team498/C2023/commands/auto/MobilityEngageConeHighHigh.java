package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.drivetrain.SimpleDrive;
import org.team498.C2023.commands.drivetrain.chargestation.AutoEngageBangBang;
import org.team498.C2023.commands.drivetrain.chargestation.BangBangBalance;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.commands.robot.GroundIntake;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MobilityEngageConeHighHigh implements Auto {
    private final Gyro gyro = Gyro.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CONE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                new FullScore(),
                new InstantCommand(() -> drivetrain.setAngleGoal(180 - Robot.rotationOffset)),
                new ParallelRaceGroup(
                        new PathPlannerFollower(PathLib.fourthNodeToSecondCube),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new GroundIntake()
                        )
                ),
                new ParallelCommandGroup(
                        new PathPlannerFollower(PathLib.secondCubeToFifthNode),
                        new ReturnToIdle()
                ),
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                new FullScore(),
                new AutoEngageBangBang()
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.fifthNodeToSecondCube.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }

}
