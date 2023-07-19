package org.team498.C2023.commands.drivetrain.chargestation;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToTip extends CommandBase {
    private Gyro gyro = Gyro.getInstance();
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private boolean onStation;
    private boolean isLevel;
    private double deadzone;
    private boolean invert;

    public DriveToTip(boolean isGridSide) {
        deadzone = 15;
        this.invert = isGridSide;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        onStation = false;
        isLevel = false;
        drivetrain.setAngleGoal(180 - Robot.rotationOffset);
    }

    @Override
    public void execute() {
        double angle = gyro.getPitch();
        if (Math.abs(angle) > deadzone) {
            onStation = true;
        }
        if (onStation && Math.abs(angle) < 14) {
            isLevel = true;
        } 
        
        if (!isLevel) {
            drivetrain.drive((-3 * Robot.coordinateFlip) * (invert ? -1 : 1), 0, drivetrain.calculateAngleSpeed(), true);
        } else {
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return isLevel;
    }
}
