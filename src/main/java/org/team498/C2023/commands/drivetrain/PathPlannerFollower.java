package org.team498.C2023.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.util.PoseUtil;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class PathPlannerFollower extends Command {
    private final Drivetrain drivetrain;
    private final PathPlannerTrajectory trajectory;
    private final Timer trajectoryTimer = new Timer();
    private final Timer stopTimer = new Timer();

    private final Map<Double, Double> stopPoints = new HashMap<>();
    private final LinkedList<Double> stopTimes = new LinkedList<>();
    private final List<Pose2d> markerPoses = new LinkedList<>();

    public PathPlannerFollower(PathPlannerTrajectory trajectory) {
        this.drivetrain = Drivetrain.getInstance();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        trajectoryTimer.reset();
        trajectoryTimer.start();

        for (EventMarker marker : trajectory.getMarkers()) {
            for (String event : marker.names) {
                System.out.println(event);
                if (event.startsWith("D")) {
                    if (Robot.alliance == Alliance.Blue) {
                        stopPoints.put(marker.timeSeconds, Double.parseDouble(event.substring(1)));
                        markerPoses.add(new Pose2d(marker.positionMeters, new Rotation2d()));
                    } else {
                        stopPoints.put(marker.timeSeconds, Double.parseDouble(event.substring(1)));
                        markerPoses.add(PoseUtil.flip(PoseUtil.toPose2d(marker.positionMeters)));
                    }
                }
            }
        }

        stopTimes.addAll(stopPoints.keySet());

        // Pose2d initialPose = Robot.alliance == Alliance.Blue ? trajectory.getInitialHolonomicPose() : PoseUtil.flip(trajectory.getInitialHolonomicPose());
        // drivetrain.setPose(initialPose);
        // drivetrain.setYaw(initialPose.getRotation().getDegrees());

        // Display the trajectory on the driver station dashboard
        List<Pose2d> poses = new LinkedList<>();
        List<Trajectory.State> trajectoryStates = trajectory.getStates();
        for (int i = 0; i < trajectoryStates.size(); i += trajectoryStates.size() / 60) {
            if (Robot.alliance == Alliance.Blue) {
                poses.add(trajectoryStates.get(i).poseMeters);
            } else {
                poses.add(PoseUtil.flip(trajectoryStates.get(i).poseMeters));
            }
        }

        Logger.getInstance().recordOutput("CurrentTrajectory", trajectory);

    }

    boolean hasReset = false;

    @Override
    public void execute() {
        // if (markerPoses.size() > 0) {
            // if (!RobotPosition.isNear(markerPoses.get(0), 0.25)) {
                // PathPlannerState state = (PathPlannerState) trajectory.sample(trajectoryTimer.get());
// 
                // if (Robot.alliance == Alliance.Blue) {
                    // drivetrain.setPositionGoal(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation));
                // } else {
                    // drivetrain.setPositionGoal(PoseUtil.flip(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation)));
                // }
// 
                // var speeds = drivetrain.calculatePositionSpeed();
                // drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
            // } else {
                // if (!hasReset) {
                    // trajectoryTimer.stop();
                    // drivetrain.drive(0, 0, 0, false);
                    // stopTimer.reset();
                    // stopTimer.start();
                    // hasReset = true;
                // }
// 
                // if (stopTimer.hasElapsed(stopPoints.get(stopTimes.get(0)))) {
                    // markerPoses.remove(0);
                    // stopTimes.remove(0);
                    // stopTimer.stop();
                    // trajectoryTimer.start();
                    // hasReset = false;
                // }
            // }
        // } else {
            PathPlannerState state = (PathPlannerState) trajectory.sample(trajectoryTimer.get());

            if (Robot.alliance == Alliance.Blue) {
                drivetrain.setPositionGoal(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation));
            } else {
                drivetrain.setPositionGoal(PoseUtil.flip(new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), state.holonomicRotation)));
            }

            var speeds = drivetrain.calculatePositionSpeed();
            drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
        // }
    }

    @Override
    public boolean isFinished() {
        // return trajectoryTimer.get() > trajectory.getTotalTimeSeconds();
        if (Robot.alliance == Alliance.Blue) {
            return RobotPosition.isNear(trajectory.getEndState().poseMeters, 0.1);
        } else {
            return RobotPosition.isNear(PoseUtil.flip(trajectory.getEndState().poseMeters), 0.1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
