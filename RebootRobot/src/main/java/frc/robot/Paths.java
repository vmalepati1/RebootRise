package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.drivetrain;

public class Paths {

    public static class TestTrajectories {

        public static Trajectory testForward = generateTestForward();

        public static Trajectory generateTestForward() {
            TrajectoryConfig config = new TrajectoryConfig(
                    Units.feetToMeters(11), Units.feetToMeters(5));
            config.addConstraint(
                    new DifferentialDriveVoltageConstraint(drivetrain.getDrivetrainFeedforward(),
                            drivetrain.getDriveKinematics(), 10.0));
            config.setKinematics(drivetrain.getDriveKinematics());

            return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(
                            new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(0.0)),
                            new Pose2d(Units.feetToMeters(15), Units.feetToMeters(0),
                                    Rotation2d.fromDegrees(0.0))),
                    config
            );
        }
    }

}
