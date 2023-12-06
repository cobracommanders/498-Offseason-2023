package org.team498.C2023.commands.robot;

import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.commands.drivetrain.DriveToPosition;
import org.team498.C2023.subsystems.vision.Vision;
import org.team498.lib.util.PoseUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCollectCone extends SequentialCommandGroup {
    static double x = Units.inchesToMeters(610);
    static double y = Units.inchesToMeters(280);

    public AutoCollectCone(){
        super(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new DriveToPosition(()-> (Robot.alliance == Alliance.Blue? new Pose2d(x - 1, y, new Rotation2d()): PoseUtil.flip(new Pose2d(x - 1, y, new Rotation2d())))),
                    new DriveToPosition(()-> (Robot.alliance == Alliance.Blue? new Pose2d(x, y, new Rotation2d()): PoseUtil.flip(new Pose2d(x, y, new Rotation2d()))))
                ),
                new WaitCommand(0),
                ()-> Vision.getInstance().rightInputs.connected && RobotPosition.inLoadingZone())
        );
    }
}
