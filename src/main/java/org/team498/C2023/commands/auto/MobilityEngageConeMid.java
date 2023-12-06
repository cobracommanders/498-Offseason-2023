package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.drivetrain.SimpleDrive;
import org.team498.C2023.commands.drivetrain.chargestation.BangBangBalance;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MobilityEngageConeMid implements Auto {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CONE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.AUTO_MID_CONE)),
                new FullScore(),
                new InstantCommand(() -> drivetrain.setAngleGoal(180 - Robot.rotationOffset)),
                new ParallelRaceGroup(
                        new SimpleDrive(() -> -2 * Robot.coordinateFlip, () -> 0,
                                () -> Drivetrain.getInstance().calculateAngleSpeed()),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Math.abs(drivetrain.getPitch()) > 6),
                                new WaitUntilCommand(() -> Math.abs(drivetrain.getPitch()) < .75),
                                new WaitCommand(0.2)
                                )).withTimeout(3),
                new SimpleDrive(()-> 0, ()-> 0, ()-> 0).withTimeout(1),
                new ParallelRaceGroup(
                        new SimpleDrive(() -> 1.5 * Robot.coordinateFlip, () -> 0,
                                () -> Drivetrain.getInstance().calculateAngleSpeed()),
                        new SequentialCommandGroup(
                                new WaitCommand(0.15),
                                new WaitUntilCommand(() -> Math.abs(drivetrain.getPitch()) > 10),
                                new WaitCommand(0.5),
                                new InstantCommand(drivetrain::stop))),
                new BangBangBalance()
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.mobilityEngage.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }

}
