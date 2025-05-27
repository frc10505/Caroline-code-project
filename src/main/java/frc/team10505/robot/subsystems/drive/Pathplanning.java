package frc.team10505.robot.subsystems.drive;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team10505.robot.subsystems.drive.Poses.ReefSpot;
import static frc.team10505.robot.subsystems.drive.Poses.Waypoints.*;

public class Pathplanning {

    private PathConstraints constraints = new PathConstraints(3.0,
            3.0, Units.degreesToRadians(180), Units.degreesToRadians(180));

    private List<Waypoint> getWaypoints(ReefSpot spot, Pose2d currentPose) {
        if (spot == ReefSpot.A) {
            return station_ToReefA;
        } else if (spot == ReefSpot.B) {
            return station_ToReefB;
        } else if (spot == ReefSpot.C && currentPose.getY() < 4) {
            return rightStation_ToReefC;
        } else if (spot == ReefSpot.C) {
            return leftStation_ToReefC;
        } else if (spot == ReefSpot.D && currentPose.getY() < 4) {
            return rightStation_ToReefD;
        } else if (spot == ReefSpot.D) {
            return leftStation_ToReefD;
        }

        else {
            return station_ToReefB;
        }

    }

    public Command goToBarge(Pose2d currentPose){
        List<Waypoint> waypoints;
        if(currentPose.getX() < 4 && currentPose.getY() > 4){
            waypoints = frontLeftReef_ToBarge;
        } else if (currentPose.getX() < 4){
            waypoints = frontRightReef_ToBarge;
        } else if (currentPose.getY() < 4){
            waypoints = rightReef_ToBarge;
        } else {
            waypoints = backReef_ToBarge;
        }


        try {
            PathPlannerPath flypath = new PathPlannerPath(waypoints, constraints, null,
                    new GoalEndState(0.0, Rotation2d.fromDegrees(180)));
            flypath.preventFlipping = true;

            return AutoBuilder.followPath(flypath);
        } catch (Exception e) {
            SmartDashboard.putString("Errors", "Flypath failed!!");
            return none();
        }
    }

    public Command goToReef(ReefSpot spot, double endRotation, Pose2d currentPose) {
        List<Waypoint> waypoints = getWaypoints(spot, currentPose);

        try {
            PathPlannerPath flypath = new PathPlannerPath(waypoints, constraints, null,
                    new GoalEndState(0.0, Rotation2d.fromDegrees(endRotation)));
            flypath.preventFlipping = true;

            return AutoBuilder.followPath(flypath);
        } catch (Exception e) {
            SmartDashboard.putString("Errors", "Flypath failed!!");
            return none();
        }
    }
}