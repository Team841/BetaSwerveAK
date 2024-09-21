package com.team841.calliope.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetection extends SubsystemBase {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable poseTest = inst.getTable("Pose Test");

    StructTopic<Pose2d> limeT = poseTest.getStructTopic("Limelight Pose", Pose2d.struct);
    StructTopic<Pose2d> noteT = poseTest.getStructTopic("Note Pose", Pose2d.struct);

    StructPublisher<Pose2d> limeP = limeT.publish();
    StructPublisher<Pose2d> noteP = noteT.publish();

    // Constants
    private static final double CAMERA_HEIGHT = 0.51; // Height of the camera from the ground in meters
    private static final double TARGET_HEIGHT = 0.051; // Height of the target from the ground in meters

    private final Rotation2d cameraPitch = new Rotation2d(0);

    private final Pose2d cameraPose = new Pose2d(new Translation2d(10, 5), new Rotation2d());

    public static Pose2d calculateTargetPose(double tx, double ty, double ta) {
        // Convert angles to radians
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);

        // Estimate the distance to the target using vertical angle and heights
        double distance = estimateDistanceToTarget(tyRad);

        // Calculate x, y, z coordinates
        double z = distance;
        double x = distance * Math.tan(txRad);
        double y = z * Math.tan(tyRad);

        // Calculate the rotation based on the horizontal angle
        Rotation2d rotation = new Rotation2d(txRad);

        // Create Translation2d
        Translation2d translation = new Translation2d(x, y);

        // Combine translation and rotation into Pose2d
        Pose2d pose = new Pose2d(translation, rotation);

        return pose;
    }

    private Translation2d getCameraToGoalTranslation(double xi, double yi, double zi) {
        Translation2d xz_plane_translation = new Translation2d(xi, zi).rotateBy(cameraPitch);
        double x = xz_plane_translation.getX();
        double y = yi;
        double z = xz_plane_translation.getY();

        // find intersection with the goal
        double differential_height = TARGET_HEIGHT - CAMERA_HEIGHT;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling;
            double sin_angle_, cos_angle_;
            double kEpsilon = 1e-12;
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            } else {
                sin_angle_ = 0.0;
                cos_angle_ = 1.0;
            }
            Rotation2d angle = new Rotation2d(cos_angle_, sin_angle_);
            return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
        }
        return null;
    }


    private static double estimateDistanceToTarget(double tyRad) {
        // Use trigonometry to estimate the distance to the target
        double heightDifference = TARGET_HEIGHT - CAMERA_HEIGHT;
        return heightDifference / Math.tan(tyRad);
    }

    public NoteDetection(){
        limeP.set(cameraPose);
    }


    @Override
    public void periodic() {
        //var tx = LimelightHelpers.getTX("Pipeline_Name");
        //var ty = LimelightHelpers.getTY("Pipeline_Name");
        //var ta = LimelightHelpers.getTA("Pipeline_Name");

        var tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        var ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        //System.out.println("tx: " + tx + ", ty: " + ty);

        if (Math.abs(tx) > 0.05 && Math.abs(ty) > 0.05) {
            var trans = getCameraToGoalTranslation(1.26, Rotation2d.fromDegrees(-tx).getTan(), Rotation2d.fromDegrees(ty).getTan());
            if (trans != null){
                var output = new Pose2d(trans, new Rotation2d(0));
                SmartDashboard.putNumber("X", output.getX());
                SmartDashboard.putNumber("Y", output.getY());
                noteP.set(cameraPose.relativeTo(output));
                }
        }
    }
}
