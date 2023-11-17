package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.List;
import java.util.function.DoubleSupplier;

public final class Autos {

  private final Drivetrain drive;

  public Autos(Drivetrain drive) {
    this.drive = drive;
  }

  public Command testTrajectory() {
    return drive.followPath(
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(),
            new Pose2d(2, 2, Rotation2d.fromDegrees(15)),
            DriveConstants.kConfig));
  }

  public Command backAndForth() {
    return driveDistance(0.5, 1).andThen(turnDegrees(0.4, 180)).andThen(driveDistance(0.5, 1));
  }

  public Command driveDistance(double speed, double meters) {
    return runOnce(drive::resetEncoders, drive)
        .andThen(drive.arcadeDrive(() -> speed, () -> 0.0))
        .until(() -> Math.abs(drive.getAverageDistanceMeters()) >= meters)
        .finallyDo(() -> drive.arcadeDrive(0, 0))
        .withName("driving " + meters + " meters");
  }

  public Command turnDegrees(double speed, double degrees) {
    /* Need to convert distance travelled to degrees. The Standard
       XRP Chassis found here, https://www.sparkfun.com/products/22230,
       has a wheel placement diameter (163 mm) - width of the wheel (8 mm) = 155 mm
       or 6.102 inches. We then take into consideration the width of the tires.

       RAHHHHH METRIC UNITS
    */
    double metersPerDegree = Math.PI * Units.inchesToMeters(6.102) / 360;

    DoubleSupplier averageTurningDistance =
        () ->
            (Math.abs(drive.getLeftDistanceMeters()) + Math.abs(drive.getRightDistanceMeters()))
                / 2;

    return runOnce(drive::resetEncoders, drive)
        .andThen(drive.arcadeDrive(() -> 0, () -> speed))
        .until(() -> averageTurningDistance.getAsDouble() >= metersPerDegree * degrees)
        .finallyDo(() -> drive.arcadeDrive(0, 0))
        .withName("driving " + degrees + " degrees");
  }
}
