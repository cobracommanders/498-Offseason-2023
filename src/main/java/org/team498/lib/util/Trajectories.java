package org.team498.lib.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

public class Trajectories {
    public static Trajectory getTrajectory(String name) {
        String filepath = "pathplanner/generatedJSON/" + name + ".wpilib.json";

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            e.printStackTrace();
            return new Trajectory();
        }
    }

    public static PathPlannerTrajectory getPathPlannerTrajectory(String name) {
        // return PathPlanner.loadPath(name, 3, 2.4);
        return PathPlanner.loadPath(name, 4.6, 4.2);//max acccel 5 (4.6, 4.5)
    }
    public static PathPlannerTrajectory getPathPlannerTrajectory(String name, double maxAccel, double maxVel) {
        // return PathPlanner.loadPath(name, 3, 2.4);
        return PathPlanner.loadPath(name, maxVel, maxAccel);
    }
}
