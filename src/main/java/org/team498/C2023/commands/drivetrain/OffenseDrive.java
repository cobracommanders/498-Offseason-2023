package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class OffenseDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier slowDriveSupplier;

    public OffenseDrive(DoubleSupplier xTranslationSupplier,
                        DoubleSupplier yTranslationSupplier,
                        DoubleSupplier rotationSupplier,
                        BooleanSupplier slowDriveSupplier) {
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.slowDriveSupplier = slowDriveSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Robot.controls.driver.setRightStickLastAngle(drivetrain.getYaw() + Robot.rotationOffset);
    }

    @Override
    public void execute() {
        double speed = slowDriveSupplier.getAsBoolean()
                       ? 0.5
                       : 1;
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double rotation = rotationSupplier.getAsDouble() + Robot.rotationOffset;

        // Set the target of the PID controller
        drivetrain.setAngleGoal(rotation);

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