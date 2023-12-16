package org.team498.C2024.commands.auto;
import org.team498.C2024.PathLib;
import org.team498.C2024.commands.drivetrain.PathPlannerFollower;
import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TestAuto implements Auto{

    @Override
    public Command getCommand() {
        return new PathPlannerFollower(PathLib.choreoTest);
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.choreoTest.getInitialTargetHolonomicPose();
    }
    
}
