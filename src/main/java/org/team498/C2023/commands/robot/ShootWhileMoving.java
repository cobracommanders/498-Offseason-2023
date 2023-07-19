package org.team498.C2023.commands.robot;


import org.team498.C2023.FieldPositions;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.drivetrain.SimpleDrive;
import org.team498.C2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootWhileMoving extends SequentialCommandGroup {

    public ShootWhileMoving() {
        super(
            new InstantCommand(() -> Drivetrain.getInstance().setXGoal(Robot.alliance == Alliance.Blue ? 1.95 : FieldPositions.midline + (FieldPositions.midline - 1.95))),
            new SetRobotState(State.SHOOT_DRIVE_CUBE_MID),
            new ParallelCommandGroup(
                new PrepareToScore(),
                new SimpleDrive(Drivetrain.getInstance()::calculateXSpeed, Robot.controls.driver::leftXSquared, () -> {
                    Drivetrain.getInstance().setAngleGoal(RobotPosition.calculateDegreesToTarget(RobotPosition.getFutureScoringNodePosition()));
                    return Drivetrain.getInstance().calculateAngleSpeed();
                }))
        );
    }
}
