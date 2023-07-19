package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DriveToPosition extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Supplier<Pose2d> target;

    public DriveToPosition(Supplier<Pose2d> target) {
        this.target = target;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoal(target.get());
    }

    @Override
    public void execute() {
        var speeds = drivetrain.calculatePositionSpeed();
        
        drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atPositionGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
