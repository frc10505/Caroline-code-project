package frc.team10505.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Poses {
    public static final Pose2d reefAPose = new Pose2d(3.96, 7.4, new Rotation2d(0));// TODO guesstemated - fix
    public static final Pose2d reefBPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefCPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefDPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefEPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefFPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefGPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefHPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefIPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefJPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefKPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d reefLPose = new Pose2d(0, 0, new Rotation2d(0));

    public static final Pose2d leftStationPose = new Pose2d(0.6, 7.3, new Rotation2d(30));// TODO guesstimated - fix
    public static final Pose2d rightStationPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d bargeShotPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d processerShotPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Pose2d endgamePose = new Pose2d(0, 0, new Rotation2d(0));

    public class Waypoints {
        private List<Waypoint> station_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> station_ToReefB = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefBPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);

        private List<Waypoint> leftStation_ToReefA = PathPlannerPath.waypointsFromPoses(
                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                reefAPose);
    }
}
