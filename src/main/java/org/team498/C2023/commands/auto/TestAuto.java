package org.team498.C2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.team498.C2023.PathLib;
import org.team498.C2023.State;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;

public class TestAuto implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> Drivetrain.getInstance().setPose(getInitialPose())),
            new PathPlannerFollower(PathLib.eighthNodeToChargeStation));
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.eighthNodeToChargeStation.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }
}
