package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class TargetDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final BooleanSupplier slowDriveSupplier;
    private final Supplier<Pose2d> target;

    public TargetDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, BooleanSupplier slowDriveSupplier, Supplier<Pose2d> target) {
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.slowDriveSupplier = slowDriveSupplier;
        this.target = target;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double speed = slowDriveSupplier.getAsBoolean()
                       ? 0.5
                       : 1;
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;

        // Set the target of the PID controller
        drivetrain.setAngleGoal(RobotPosition.calculateDegreesToTarget(target.get()));

        // Calculate the rotational speed from the pid controller, unless it's already at the goal
        double rotationalSpeed = drivetrain.calculateAngleSpeed();

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotationalSpeed, true);
    }



    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}