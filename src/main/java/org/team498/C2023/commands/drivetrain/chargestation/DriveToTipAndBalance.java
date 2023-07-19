package org.team498.C2023.commands.drivetrain.chargestation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveToTipAndBalance extends SequentialCommandGroup {
    public DriveToTipAndBalance() {
        super(
            new DriveToTip(false),
            new WaitCommand(1),
            new BangBangBalance()
        );
    }
}
