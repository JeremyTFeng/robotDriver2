package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.robotcore.internal.system.Assert.assertTrue;

// wheel to Robot
public class ForwardKinematics {
    public static Pose2d wheelToRobotVelocities(List<Double> wheelVelocities){
        double lateralMultiplier = 1.0;
        double frontLeft = 0, rearLeft = 0, rearRight = 0, frontRight = 0;
        if (wheelVelocities.size() == 4) {
            frontLeft = wheelVelocities.get(0);
            rearLeft = wheelVelocities.get(1);
            rearRight = wheelVelocities.get(2);
            frontRight = wheelVelocities.get(3);
        }
        else if (wheelVelocities.size() == 2) {
            frontLeft = wheelVelocities.get(0);
            rearLeft = wheelVelocities.get(0);
            rearRight = wheelVelocities.get(1);
            frontRight = wheelVelocities.get(1);
        }
        else
            assertTrue(true);
        double k = (DriveConstants.TRACK_WIDTH + DriveConstants.WHEEL_BASE) / 2.0;
        Pose2d r = new Pose2d(rearLeft + frontRight + frontLeft + rearRight,
                (rearLeft + frontRight - frontLeft - rearRight) / lateralMultiplier,
                (rearRight + frontRight - frontLeft - rearLeft) / k);
        r = r.times(0.25);
        RobotLogger.dd("ForwardKinematics", "wheelToRobotVelocities " + r.toString());
        return (r);
    }


    public static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.getHeading();
        double sinTerm, cosTerm;
        if (Angle.epsilonEquals(dtheta,  0.0)) {
            sinTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sinTerm = sin(dtheta) / dtheta;
            cosTerm = (1 - cos(dtheta)) / dtheta;
        }

        Vector2d fieldPositionDelta = new Vector2d(
                sinTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(),
                cosTerm * robotPoseDelta.getX() + sinTerm * robotPoseDelta.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotated(fieldPose.getHeading()), robotPoseDelta.getHeading());
        Pose2d r = new Pose2d(
                fieldPose.getX() + fieldPoseDelta.getX(),
                fieldPose.getY() + fieldPoseDelta.getY(),
                Angle.norm(fieldPose.getHeading() + fieldPoseDelta.getHeading()));
        RobotLogger.dd("ForwardKinematics", "Kinematics: relativeOdometryUpdate, fieldPoseDelta " + fieldPoseDelta.toString()
                + " new pose: " + r.toString());

        return (r);
    }

    public double calculateMotorFeedforward(double vel, double accel) {
        double basePower = vel * DriveConstants.kV + accel * DriveConstants.kA;
        RobotLogger.dd("ForwardKinematics", "accel", "Kinematics: calculateMotorFeedforward: kV " +
                Double.toString(DriveConstants.kV) + " kA: " + Double.toString(DriveConstants.kA));
        RobotLogger.dd("ForwardKinematics", "vel: " + Double.toString(vel) + " accel: " + Double.toString(accel));
        if (Angle.epsilonEquals(basePower, 0.0)) {
            return 0.0;
        } else {
            return (basePower + Math.signum(basePower) * DriveConstants.kStatic);
        }
    }
}
