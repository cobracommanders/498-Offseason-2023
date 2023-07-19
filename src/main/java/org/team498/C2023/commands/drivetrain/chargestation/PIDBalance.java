package org.team498.C2023.commands.drivetrain.chargestation;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDBalance extends CommandBase {
    private Gyro gyro = Gyro.getInstance();
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private boolean isTipped;
    private double deadzone;
    private boolean invert;
    private PIDController PID = new PIDController(0.5, 0, 0);

    public PIDBalance(boolean invert) {
        deadzone = 8;
        this.invert = invert;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isTipped = false;
        PID.setSetpoint(0);
        drivetrain.setAngleGoal(180 - Robot.rotationOffset);
    }

    @Override
    public void execute() {
        double angle = gyro.getPitch();
        if (Math.abs(angle) > deadzone) {
            isTipped = true;
            drivetrain.drive(Math.copySign(PID.calculate(gyro.getPitch()), -angle * Robot.coordinateFlip), 0, drivetrain.calculateAngleSpeed(), true);
        } else if (!isTipped) {
            drivetrain.drive((-3 * Robot.coordinateFlip) * (invert ? -1 : 1), 0, drivetrain.calculateAngleSpeed(), true);
        } else {
            drivetrain.drive(0, 0, 0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
