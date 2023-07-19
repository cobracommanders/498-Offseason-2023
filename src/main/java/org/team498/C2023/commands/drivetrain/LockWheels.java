package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.subsystems.Drivetrain;

public class LockWheels extends InstantCommand {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    @Override
    public void initialize() {
        drivetrain.X();
    }
}
