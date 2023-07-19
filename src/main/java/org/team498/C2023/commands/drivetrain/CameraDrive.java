package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.DrivetrainConstants.AngleConstants.MAX_ANGULAR_SPEED_DEGREES_PER_SECOND;
import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class CameraDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;

    public CameraDrive(DoubleSupplier xTranslationSupplier, DoubleSupplier yTranslationSupplier, DoubleSupplier rotationSupplier) {
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND;
        double rotation = rotationSupplier.getAsDouble() * MAX_ANGULAR_SPEED_DEGREES_PER_SECOND;

        drivetrain.drive(xTranslation, yTranslation, rotation, false);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}