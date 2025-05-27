package frc.team10505.robot.subsystems.drive;

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

        public enum ReefSpot {
                A,
                B,
                C,
                D,
                E,
                F,
                G,
                H,
                I,
                J,
                K,
                L
        }

        public class Waypoints {
                public static final List<Waypoint> frontLeftReef_ToBarge = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                                reefAPose);

                public static final List<Waypoint> frontRightReef_ToBarge = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                                reefAPose);

                public static final List<Waypoint> rightReef_ToBarge = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                                reefAPose);

                public static final List<Waypoint> backReef_ToBarge = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                                reefAPose);

                

                public static final List<Waypoint> station_ToReefA = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.4, new Rotation2d(0)),
                                reefAPose);

                public static final List<Waypoint> station_ToReefB = PathPlannerPath.waypointsFromPoses(
                                new Pose2d(3.8, 7.35, new Rotation2d(0)),
                                reefBPose);

                public static final List<Waypoint> rightStation_ToReefC = PathPlannerPath.waypointsFromPoses(
                                reefCPose);

                public static final List<Waypoint> leftStation_ToReefC = PathPlannerPath.waypointsFromPoses(
                                reefCPose);

                public static final List<Waypoint> rightStation_ToReefD = PathPlannerPath.waypointsFromPoses(
                                reefDPose);

                public static final List<Waypoint> leftStation_ToReefD = PathPlannerPath.waypointsFromPoses(
                                reefDPose);

                public static final List<Waypoint> rightStation_ToReefE = PathPlannerPath.waypointsFromPoses(
                                reefEPose);

                public static final List<Waypoint> rightStation_ToReefF = PathPlannerPath.waypointsFromPoses(
                                reefFPose);

                public static final List<Waypoint> rightStation_ToReefG = PathPlannerPath.waypointsFromPoses(
                                reefGPose);

                public static final List<Waypoint> leftStation_ToReefH = PathPlannerPath.waypointsFromPoses(
                                reefHPose);

                public static final List<Waypoint> leftStation_ToReefI = PathPlannerPath.waypointsFromPoses(
                                reefIPose);

                public static final List<Waypoint> leftStation_ToReefJ = PathPlannerPath.waypointsFromPoses(
                                reefJPose);

                public static final List<Waypoint> leftStation_ToReefK = PathPlannerPath.waypointsFromPoses(
                                reefKPose);

                public static final List<Waypoint> leftStation_ToReefL = PathPlannerPath.waypointsFromPoses(
                                reefLPose);
        }
}
