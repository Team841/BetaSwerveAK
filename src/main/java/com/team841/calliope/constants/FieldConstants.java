package com.team841.calliope.constants;

import com.team841.lib.util.FieldGeometry;
import com.team841.lib.util.dyn4jBridge;
import edu.wpi.first.math.geometry.*;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class FieldConstants {
    public static final double FIELD_WIDTH = 16.54;
    public static final double FIELD_HEIGHT = 8.21;
    public static Pose3d kBlueSpeakerPose3d = new Pose3d(0.0, 5.549, 2.002, new Rotation3d());
    public static Pose2d kBlueSpeakerPose2d =
            new Pose2d(kBlueSpeakerPose3d.getX(), kBlueSpeakerPose3d.getY(), new Rotation2d(0.0));

    public static Pose2d kRedSpeakerPose2d = FieldGeometry.flipFieldPose(kBlueSpeakerPose2d);

    public static Pose2d S_OSpeakerC =
            new Pose2d(1.3865381479263306, 4.631037712097168, new Rotation2d(3.14159));

    /**
     * stores the obstacles on a competition field, which includes the border and the game pieces
     * adapted from Maple-Swerve-Skeleton/example/5516-2024-OffSeason
     */
    public static class FieldObstaclesMap {
        List<Body> obstacles = null;

        protected Body addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
            return getObstacle(Geometry.createSegment(
                    new Vector2(startingPoint.getX(), startingPoint.getY()),
                    new Vector2(endingPoint.getX(), endingPoint.getY())));
        }

        protected Body addRectangularObstacle(double width, double height, Pose2d pose) {
            final Body obstacle = getObstacle(Geometry.createRectangle(
                    width, height
            ));

            obstacle.getTransform().set(dyn4jBridge.PoseToTransfrom(pose));
            return obstacle;
        }

        private Body getObstacle(Convex shape) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(shape);
            fixture.setFriction(0.6);
            fixture.setRestitution(0.6);
            return obstacle;
        }

        public List<Body> getObstacles() {
            if (obstacles!=null){
                return obstacles;
            } else {
                obstacles = new ArrayList<Body>();
            }

            // Left wall
            obstacles.add(addBorderLine(new Translation2d(0, 1), new Translation2d(0, 4.51)));
            obstacles.add(addBorderLine(
                    new Translation2d(0, 4.51),
                    new Translation2d(0.9,5)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(0.9, 5),
                    new Translation2d(0.9,6.05)
            ));
            obstacles.add( addBorderLine(
                    new Translation2d(0.9, 6.05),
                    new Translation2d(0,6.5)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(0, 6.5),
                    new Translation2d(0,8.2)
            ));
            // upper wall
            obstacles.add(addBorderLine(
                    new Translation2d(0, 8.12),
                    new Translation2d(FIELD_WIDTH, 8.12)
            ));
            // righter wall
            obstacles.add( addBorderLine(
                    new Translation2d(FIELD_WIDTH, 1),
                    new Translation2d(FIELD_WIDTH, 4.51)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(FIELD_WIDTH, 4.51),
                    new Translation2d(FIELD_WIDTH-0.9, 5)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(FIELD_WIDTH-0.9, 5),
                    new Translation2d(FIELD_WIDTH-0.9, 6.05)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(FIELD_WIDTH-0.9, 6.05),
                    new Translation2d(FIELD_WIDTH, 6.5)
            ));
            obstacles.add(addBorderLine(
                    new Translation2d(FIELD_WIDTH, 6.5),
                    new Translation2d(FIELD_WIDTH, 8.2)
            ));
            // lower wall
            obstacles.add(addBorderLine(
                    new Translation2d(1.92, 0),
                    new Translation2d(FIELD_WIDTH-1.92, 0)
            ));
            // red source wall
            obstacles.add(addBorderLine(
                    new Translation2d(1.92, 0),
                    new Translation2d(0,1)
            ));
            // blue source wall
            obstacles.add(addBorderLine(
                    new Translation2d(FIELD_WIDTH-1.92, 0),
                    new Translation2d(FIELD_WIDTH, 1)
            ));
            // blue state
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(3.4, 4.1,new Rotation2d())
            ));
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(5.62, 4.1-1.28,Rotation2d.fromDegrees(30))
            ));
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(5.62, 4.1+1.28,Rotation2d.fromDegrees(60))
            ));
            // red stage
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(FIELD_WIDTH-3.4, 4.1,new Rotation2d())
            ));
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(FIELD_WIDTH-5.62, 4.1-1.28,Rotation2d.fromDegrees(60))
            ));
            obstacles.add(addRectangularObstacle(
                    0.35,0.35,
                    new Pose2d(FIELD_WIDTH-5.62, 4.1+1.28,Rotation2d.fromDegrees(30))
            ));

            return obstacles;
        }
    }
}
