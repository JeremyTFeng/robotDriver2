package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to test simple drive capabilities with setPower and getWheelPositions
 */
@Config
@Autonomous(group = "drive")
//@Disabled
public class SimpleDrive extends LinearOpMode {
    private String TAG = "SimpleDrive";

    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap, this);

        FieldDashboard fieldDashboard = new FieldDashboard(drive);
        Pose2d currentPose = drive.getPoseEstimate();
        double currentX = -currentPose.getY();
        double currentY = currentPose.getX();
        double currentHeading = currentPose.getHeading();
        waitForStart();
        drive.setOpmode(this);
        while (opModeIsActive()) {
            //FL, BL, BR, FR
            //drive.setMotorPowers(0.0, 0.8, 0.8, 0.0);
            drive.setMotorPowers(0.8, 0.8);

            List<Double> wheelPositions = drive.getWheelPositions();


            currentPose = drive.getPoseEstimate();
            drive.update();
            currentX = -currentPose.getY();
            currentY = currentPose.getX();
            currentHeading = currentPose.getHeading();

            //Simulating behavior or real IMU--range of headings is +/- 180, North is 0
            while (currentHeading < -Math.PI) {
                currentHeading = currentHeading + (2 * Math.PI);
            }
            while (currentHeading > Math.PI) {
                currentHeading = currentHeading - (2 * Math.PI);
            }


            RobotLogger.dd(TAG, "localizer: (" + currentX + ", " + currentY + ", " + currentHeading + ")");
            fieldDashboard.updateDashboard();

            SafeSleep.sleep_milliseconds(this, 50);
        }
    }
}