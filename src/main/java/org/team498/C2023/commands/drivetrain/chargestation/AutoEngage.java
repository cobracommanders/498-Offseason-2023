package org.team498.C2023.commands.drivetrain.chargestation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.util.PoseUtil;

public class AutoEngage extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final double chargingStationPoseX = 3.93;
    private final boolean yEnabled;
    private final double yPos;

    public AutoEngage() {
        addRequirements(drivetrain);
        yEnabled = false;
        yPos = 0.0;
    }

    public AutoEngage(double y) {
        addRequirements(drivetrain);
        yEnabled = true;
        yPos = y;
    }

    @Override
    public void initialize() {
        drivetrain.setPositionGoal(Robot.alliance == Alliance.Blue ? new Pose2d(chargingStationPoseX, yPos, Rotation2d.fromDegrees(180)) : PoseUtil.flip(new Pose2d(chargingStationPoseX, 0, Rotation2d.fromDegrees(180))));
    }

    @Override
    public void execute() {
        drivetrain.drive(drivetrain.calculateXSpeed(), yEnabled ? drivetrain.calculateYSpeed() : 0, drivetrain.calculateAngleSpeed(), true);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.atXGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
