package org.team498.C2024.commands.drivetrain;

import org.team498.C2024.Robot;
import org.team498.C2024.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.team498.C2024.Constants.DrivetrainConstants.AngleConstants.MAX_ANGULAR_SPEED_DEGREES_PER_SECOND;
import static org.team498.C2024.Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;

public class HybridDrive extends Command {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private final DoubleSupplier xTranslationSupplier;
    private final DoubleSupplier yTranslationSupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier slowDriveSupplier;
    private final DoubleSupplier povAngle;
    private double desiredAngle;

    public HybridDrive(DoubleSupplier xTranslationSupplier,
                        DoubleSupplier yTranslationSupplier,
                        DoubleSupplier rotationSupplier,
                        DoubleSupplier povAngle,
                        BooleanSupplier slowDriveSupplier) {
        this.xTranslationSupplier = xTranslationSupplier;
        this.yTranslationSupplier = yTranslationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.slowDriveSupplier = slowDriveSupplier;
        this.povAngle = povAngle;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        desiredAngle = drivetrain.getYaw();
    }

    @Override
    public void execute() {
        double speed = slowDriveSupplier.getAsBoolean()
                       ? 0.5
                       : 1;
        double xTranslation = xTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double yTranslation = yTranslationSupplier.getAsDouble() * MAX_VELOCITY_METERS_PER_SECOND * Robot.coordinateFlip * speed;
        double rotation = Math.copySign(Math.pow(rotationSupplier.getAsDouble(), 2), rotationSupplier.getAsDouble()) * MAX_ANGULAR_SPEED_DEGREES_PER_SECOND * speed;

        if(rotation != 0){
            double rotVel = rotation * Robot.DEFAULT_PERIOD * 10;//Math.copySign(Math.sqrt(Math.abs(rotation)), rotation) * Robot.DEFAULT_PERIOD * 110;
            if (Math.abs(rotation) <= 0.3) {
                rotVel = rotation * Robot.DEFAULT_PERIOD * 5;
            }
            desiredAngle = drivetrain.getYaw() + rotVel;
        }
        if(povAngle.getAsDouble() != -1) {
            desiredAngle = povAngle.getAsDouble() - Robot.rotationOffset;
        }
        drivetrain.setAngleGoal(desiredAngle);
        //}
        //if(!drivetrain.atAngleGoal()){//rotation == 0 && (xTranslation != 0 || yTranslation != 0) || drivetrain.angle){
            //drivetrain.setAngleGoal(desiredAngle);
        rotation = drivetrain.calculateAngleSpeed();
        // } else {
        //     rotation = 0;
        // }
    

        // Set the robot to drive in field relative mode, with the rotation controlled by the snap controller
        drivetrain.drive(xTranslation, yTranslation, rotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}