package org.team498.C2023.commands.drivetrain.chargestation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoEngageBangBang extends SequentialCommandGroup {
    public AutoEngageBangBang() {
        super(
            new AutoEngage(),
            new BangBangBalance()
        );
    }

    public AutoEngageBangBang(double y) {
        super(
            new AutoEngage(y),
            new BangBangBalance()
        );
    }
}
