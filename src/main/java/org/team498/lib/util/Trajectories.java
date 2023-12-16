package org.team498.lib.util;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

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
    public static PathPlannerTrajectory getChoreoTrajectory(String name){
        return PathPlannerPath.fromChoreoTrajectory(name).getTrajectory(null, null);
    }   
    //Get trajectory doesnt require starting speeds/rotation cause of Choreo
//    //public static PathPlannerTrajectory getPathPlannerTrajectory(String name) {
//         // return PathPlanner.loadPath(name, 3, 2.4);
//         return PathPlannerPath.loadPath(name, 3, 2);
//         PathPlannerAuto.getPathGroupFromAutoFile().
//     }
}
