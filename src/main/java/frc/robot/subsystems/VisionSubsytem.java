// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsytem extends SubsystemBase {
  private static final int OUT_OF_ROT_TOLERANCE_DEBOUNCE = 4;
  private static final int LOST_TARGET_DEBOUNCE = 20;

  boolean inRotTolerance = false;
  int outOfRotToleranceDebounceCount = OUT_OF_ROT_TOLERANCE_DEBOUNCE;
  boolean inLatTolerance = false;

  DriveSubsystem m_driveSubsytem;

  LimelightTarget_Fiducial leftTag;
  LimelightTarget_Fiducial rightTag;

  Pose3d rightAprilTransform3d;
  Optional<Pose3d> leftAprilTransform3d;
  Optional<Pose3d> targetPose;

  boolean rightTargetIDValid;
  boolean leftTargetIDValid;

  double cameraOffset;
  double distance;
  double xSpeed;
  double calcRot;
  double rot;
  double ySpeed;

  int lostTargetDebounceCount;
  int runningAverage = 2;

  List<Double> averageDistance = new ArrayList<Double>();
  List<Double> averageLateral = new ArrayList<Double>();

  AprilTagFieldLayout m_aprilTagField;

  Field2d m_field = new Field2d();

  public VisionSubsytem(DriveSubsystem driveSubsystem) {
    m_driveSubsytem = driveSubsystem;

    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ?
        OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException err) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", err.getStackTrace());
      layout = null;
    }
    this.m_aprilTagField = layout;

    SmartDashboard.putData("Field", m_field);
  }

  private LimelightTarget_Fiducial getClosestTag(String cameraName) {
    double closest = 100;
    LimelightTarget_Fiducial target = null;
    LimelightTarget_Fiducial[] targetList = LimelightHelpers.getLatestResults(cameraName).targetingResults.targets_Fiducials;
    for (LimelightTarget_Fiducial i : targetList) {
      double value = i.tx;
      if (value < closest) {
        closest = value;
        target = i;
      }
    }
    return target;
  }

  public LimelightTarget_Fiducial getTagByID(LimelightTarget_Fiducial[] targets, double ID) {
    for (LimelightTarget_Fiducial t : targets) {
      if (t.fiducialID == ID) {
        return t;
      }
    }
    return null;
  }

  public double getAverage(List<Double> list) {
    double total = 0;
    for (Double element : list) {
      total += element;
    }
    return (total / list.size());
  }

  public Command centerAprilTagCommand(final double backOffset) {
    return new CenterAprilTag(backOffset);
  }

  private void centerApriltag(final double backOffset) {
    LimelightResults leftResult = LimelightHelpers.getLatestResults("limelight-right");

    var leftClosestTag = getClosestTag("limelight-right");
    if (leftResult.targetingResults.valid && leftClosestTag != null) {
      leftTag = getTagByID(leftResult.targetingResults.targets_Fiducials, leftClosestTag.fiducialID);

      if (leftTag != null) {
        leftTargetIDValid = true;
        leftAprilTransform3d = m_aprilTagField.getTagPose((int)leftTag.fiducialID);
      } else {
        leftTargetIDValid = false;
        leftAprilTransform3d = null;
      }
    } else {
      leftTargetIDValid = false;
    }

    // LimelightResults rightResults = LimelightHelpers.getLatestResults("limelightRight");

    // var rightClosestTag = getClosestTag("limelightRight");
    // if (rightResults.targetingResults.valid && rightClosestTag != null) {
    //   rightTag = getTagByID(rightResults.targetingResults.targets_Fiducials, rightClosestTag.fiducialID);

    //   if (rightTag != null) {
    //     rightTargetIDValid = true;
    //     rightAprilTransform3d = rightTag.getRobotPose_FieldSpace();
    //   } else {
    //     rightTargetIDValid = false;
    //     rightAprilTransform3d = null;
    //   }
    // } else {
    //   rightTargetIDValid = false;
    // }

    // if (rightTargetIDValid) {
    //   targetPose = rightAprilTransform3d;
    //   cameraOffset = Constants.RIGHT_CAMERA_OFFSET_RIGHT - aprilTagOffset;
    // }

    if (leftTargetIDValid) {
      targetPose = leftAprilTransform3d;
    } else {
      targetPose = null;
    }

    if (targetPose != null) {
      Translation2d targetTransform = new Translation2d(targetPose.get().getX(), targetPose.get().getY());
      
      PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(m_driveSubsytem.getTranslation2d(), Rotation2d.fromDegrees(180), m_driveSubsytem.getRotation2d()),
        new PathPoint(targetTransform, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)));

      m_field.getObject("traj").setTrajectory(traj);

      Logger.getInstance().recordOutput("Desired X", targetTransform.getX());
      Logger.getInstance().recordOutput("Desired Y", targetTransform.getY());

      m_driveSubsytem.followTrajectoryCommand(traj, false).schedule();

    } else {
      lostTargetDebounceCount++;
    }
  }

  public void updateOdometry() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight-right");
    if (getClosestTag("limelight-right") != null) {
      m_driveSubsytem.addVision(results);
      Logger.getInstance().recordOutput("updating with tags", true);
    } else {
      Logger.getInstance().recordOutput("updating with tags", false);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  private class CenterAprilTag extends CommandBase {
    private final double m_backOffset;

    private CenterAprilTag(final double backOffset) {
      m_backOffset = backOffset;
      addRequirements(m_driveSubsytem);
    }

    @Override
    public void initialize() {
      inLatTolerance = false;
      inRotTolerance = false;
      outOfRotToleranceDebounceCount = OUT_OF_ROT_TOLERANCE_DEBOUNCE;
      averageDistance.clear();
      averageLateral.clear();
      // System.out.println("DBWADIBWADUBDWAB\n\n\n");
    }

    @Override
    public void execute() {
      centerApriltag(m_backOffset);
      // SmartDashboard.putBoolean("driving", true);
    }

    @Override
    public boolean isFinished() {
      // TODO check for drive stall. That is, up against
      // substation wall or grid edges.
      if (lostTargetDebounceCount >= LOST_TARGET_DEBOUNCE) {
        System.out.println("DIWADDAWDWADA\n\n\n\n");
        return true;
      } else if (inLatTolerance && inRotTolerance) {
        System.out.println("dYUDVWAYDVWA\n\n\n\n");
        return true;
      }
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      m_driveSubsytem.stop();
    }
  }
}