package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.elevator.Elevator;

public class ManualElevator extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final DoubleSupplier speedSupplier;

    public ManualElevator(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setManual(speedSupplier.getAsDouble() * 0.25);        
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setManual(0);
    }
}
