package com.team841.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

public class dyn4jBridge {

    public static Vector2 TranslationToVector(Translation2d translation2d){
        return new Vector2(translation2d.getX(), translation2d.getY());
    }

    public static Vector3 TranslationToVector(Translation3d translation3d){
        return new Vector3(translation3d.getX(), translation3d.getY(), translation3d.getZ());
    }

    public static Transform PoseToTransfrom(Pose2d pose2d){
        final Transform transform = new Transform();
        transform.setTranslation(TranslationToVector(pose2d.getTranslation()));
        transform.setRotation(pose2d.getRotation().getRadians());
        return transform;
    }
}
