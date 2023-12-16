package org.team498.C2024;

import org.team498.C2024.Constants.OIConstants;
import org.team498.C2024.commands.drivetrain.HybridDrive;
import org.team498.C2024.subsystems.Drivetrain;
import org.team498.lib.drivers.Xbox;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Controls {
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setDeadzone(0.15);
        driver.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
        operator.setTriggerThreshold(0.2);
    }

    public void configureDefaultCommands() {
        Drivetrain.getInstance().setDefaultCommand(new HybridDrive(driver::leftYSquared, driver::leftXSquared, driver::rightX, driver::rawPOVAngle, driver.rightBumper()));
    }

    public void configureDriverCommands() {
        driver.A().onTrue(runOnce(() -> Drivetrain.getInstance().setYaw(0)));
    }

    public void configureOperatorCommands() {

    }
}