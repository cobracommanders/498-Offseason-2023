package org.team498.C2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.FieldPositions;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.State;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.robot.ExitAutoShot;
import org.team498.lib.auto.Auto;

public class JustAutoShot implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                new SetManipulatorToNextState(),
                new WaitCommand(0.5),
                new ExitAutoShot()
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldPositions.blueCoOp.getNodePoints()[2][1].toPose2d().plus(new Transform2d(new Translation2d(RobotPosition.scoringOffset, 0), Rotation2d.fromDegrees(180)));
    }

    @Override
    public State getInitialState() {
        return State.AUTO_SHOT;
    }
}
