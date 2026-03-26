// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.io.IOException;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */

public class HubConstants {

    public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    public static boolean disableHAL = false;

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public static int[] getTagsToUse() {
        return isRedAlliance() ? new int[] { 10, 4, 2, 5 } : new int[] { 26, 20, 18, 21 };
    }

    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0);
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    public static Translation3d getTopCenterPoint() {
        double xPos = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[0]).get().getX();
        return new Translation3d(xPos + width / 2.0, fieldWidth / 2.0, height);
    }

    public static Translation3d getInnerCenterPoint() {
        double xPos = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[0]).get().getX();
        return new Translation3d(xPos + width / 2.0, fieldWidth / 2.0, innerHeight);
    }

    public static Translation2d getNearLeftCorner() {
        return new Translation2d(getTopCenterPoint().getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    }

    public static Translation2d getNearRightCorner() {
        return new Translation2d(getTopCenterPoint().getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    }

    public static Translation2d getFarLeftCorner() {
        return new Translation2d(getTopCenterPoint().getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    }

    public static Translation2d getFarRightCorner() {
        return new Translation2d(getTopCenterPoint().getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
    }

    // Hub faces
    @AutoLogOutput
    public static Pose2d getNearFace() {
        return AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[0]).get().toPose2d();
    }

    @AutoLogOutput
    public static Pose2d getFarFace() {
        return AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[1]).get().toPose2d();
    }

    @AutoLogOutput
    public static Pose2d getRightFace() {
        return AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[2]).get().toPose2d();
    }

    @AutoLogOutput
    public static Pose2d getLeftFace() {
        return AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(getTagsToUse()[3]).get().toPose2d();
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none"),
        HUB("2026-hub"),
        OUTPOST("2026-outpost"),
        TOWER("2026-tower");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            AprilTagFieldLayout layout = null;
            try {
                layout = new AprilTagFieldLayout(
                        Filesystem.getDeployDirectory().toPath().resolve("2026-official.json"));
            } catch (IOException e) {
                e.printStackTrace();
            }
            if (layout == null) {
                return null;
            } else {
                return layout;
            }
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

}
