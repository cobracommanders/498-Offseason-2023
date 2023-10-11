package org.team498.C2023.commands.drivetrain.chargestation;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BangBangBalance extends CommandBase {
    private Gyro gyro = Gyro.getInstance();
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private double deadzone;
    private double speed;

    public BangBangBalance() {
        deadzone = 7;
        speed = 1;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setAngleGoal(180 - Robot.rotationOffset);
    }

    @Override
    public void execute() {
        double angle = gyro.getPitch();
        if (Math.abs(angle) > deadzone) {
            drivetrain.drive(Math.copySign(speed, -angle * Robot.coordinateFlip), 0, drivetrain.calculateAngleSpeed(), true);
        } else {
            drivetrain.X();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
