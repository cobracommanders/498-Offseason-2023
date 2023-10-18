package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.team498.C2023.Constants.DrivetrainConstants.AngleConstants.MAX_ANGULAR_SPEED_DEGREES_PER_SECOND;
import static org.team498.C2023.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class SlowDrive extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier slowDriveSupplier;
    //private double desiredAngle;

    public SlowDrive(DoubleSupplier xTranslationSupplier,
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
        //desiredAngle = drivetrain.getYaw();
    }

    @Override
    public void execute() {
        double speed = slowDriveSupplier.getAsBoolean()
                       ? 0.5
                       : 1;
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double rotation = rotationSupplier.getAsDouble() * MAX_ANGULAR_SPEED_DEGREES_PER_SECOND * speed;
        // if(rotation != 0){
        //     double rotVel = Math.copySign(Math.sqrt(Math.abs(rotation)), rotation) * Robot.DEFAULT_PERIOD * 110;
        //     desiredAngle = drivetrain.getYaw() + rotVel;
        // }
        // if(rotation == 0 && (xTranslation != 0 || yTranslation != 0)){
        //     drivetrain.setAngleGoal(desiredAngle);
        //     rotation = drivetrain.calculateAngleSpeed();
        // }
    

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}