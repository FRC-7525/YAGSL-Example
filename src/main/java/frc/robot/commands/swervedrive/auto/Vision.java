// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

import java.lang.ModuleLayer.Controller;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class Vision extends CommandBase
{

  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final boolean          isOpenLoop;
  private final SwerveController controller;
  private final Timer            timer    = new Timer();
  private final boolean          headingCorrection;
  private       double           angle    = 0;
  private       double           lastTime = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double target = tv.getDouble(0.0);
  double botpose[] = table.getEntry("botpose").getDoubleArray(new double[6]);

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(3);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0.2);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  PhotonCamera camera = new PhotonCamera("OV5647");
  Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0));

  List<AprilTag> aprilTags = Arrays.asList(
          new AprilTag(2, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))),
          new AprilTag(5, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))));
  AprilTagFieldLayout layout = new AprilTagFieldLayout(aprilTags, 3.0, 3.0);

  PhotonPoseEstimator estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

  PIDController forwardController = new PIDController(0.9, 0, 0.0);

  final double ANGULAR_P = 0.025;
  final double ANGULAR_D = 0.0;
  PIDController angleController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  SlewRateLimiter filter = new SlewRateLimiter(0.5);
  double previous_distance;

  final Pose3d targetPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public Vision(
    SwerveSubsystem swerve, 
    DoubleSupplier vX, 
    DoubleSupplier vY, 
    DoubleSupplier omega, 
    BooleanSupplier driveMode, 
    boolean isOpenLoop, 
    boolean headingCorrection
    )
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.isOpenLoop = isOpenLoop;
    this.controller = swerve.getSwerveController();
    this.headingCorrection = headingCorrection;
    if (headingCorrection)
    {
      timer.start();
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if (headingCorrection)
    {
      lastTime = timer.get();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    var cam2d = getVisionInfo();

    if (cam2d.isPresent()) {
      swerve.drive(
        cam2d.get(),
        0.0,
        driveMode.getAsBoolean(), 
        isOpenLoop
      );
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

  public Optional<Translation2d> getVisionInfo() {
    boolean nopose;

    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      nopose = false;

      var transform = estimator.getRobotToCameraTransform();
      var cam2d = transform.getTranslation().toTranslation2d();

      return Optional.of(cam2d);
    } else {
      nopose = true;
    }

    SmartDashboard.putBoolean("No Pose?", nopose);

    return Optional.empty();
  }
}
