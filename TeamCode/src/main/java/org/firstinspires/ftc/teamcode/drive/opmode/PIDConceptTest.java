package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.SystemClock;

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
public class PIDConceptTest extends LinearOpMode {
    private String TAG = "PIDConceptTest";
    public static double DISTANCE = 48; // in

    public static double kP=10, kI=1.1, kD=5;
    private double MAX_I = 1000;

    PIDController pid_controller = new PIDController(kP, kI, kD, MAX_I);
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
        double curr_power = 0.2;
        double pid_error = 0.0;
        while (opModeIsActive()) {
            //FL, BL, BR, FR
            //drive.setMotorPowers(0.0, 0.8, 0.8, 0.0);
            double delta_power = pid_error * 0.005;   // simulate Kv
            RobotLogger.dd(TAG, "delta power: " + String.valueOf(delta_power) + "  curr_power: " + String.valueOf(curr_power));
            if (delta_power > 0.5 || delta_power < -0.5) {
                if (delta_power > 0.5) delta_power = 0.5;
                if (delta_power < -0.5) delta_power = -0.5;
                RobotLogger.dd(TAG, "max power adjustment reached");
            }
            curr_power = curr_power + delta_power;
            if (curr_power < -1.0) curr_power = -1.0;
            if (curr_power > 1.0) curr_power = 1.0;
            drive.setMotorPowers(curr_power, curr_power);

            //List<Double> wheelPositions = drive.getWheelPositions(); // implement your own localization here

            currentPose = drive.getPoseEstimate();
            drive.update();
            currentX = -currentPose.getY();
            currentY = currentPose.getX();
            currentHeading = currentPose.getHeading();
            pid_error = pid_controller.update(DISTANCE, currentY);

            //Simulating behavior or real IMU--range of headings is +/- 180, North is 0
            while (currentHeading < -Math.PI) {
                currentHeading = currentHeading + (2 * Math.PI);
            }
            while (currentHeading > Math.PI) {
                currentHeading = currentHeading - (2 * Math.PI);
            }

            RobotLogger.dd(TAG, "localizer: (" + currentX + ", " + currentY + ", " + currentHeading + ")");
            telemetry.addData("PID error", pid_error);

            fieldDashboard.updateDashboard();

            SafeSleep.sleep_milliseconds(this, 20);  // simulate control loop latency
        }
    }
}

class PIDController {
    double kP, kI, kD, MAX_I;
    long previousTime;
    double previousErr;
    double p, i, d;
    PIDController(double kP, double kI, double kD, double maxI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.MAX_I = maxI;
        previousTime = 0;
        RobotLogger.dd("PID init ", String.valueOf(this.kP)+ " " + String.valueOf(this.kI)+ " " + String.valueOf(this.kD));
    }
    double update(double target, double current) {
        if (previousTime == 0) {
            previousTime = SystemClock.elapsedRealtime();
            return 0;
        }

        long currTime = SystemClock.elapsedRealtime();
        double current_error = target - current;
        double delta_time = (currTime - previousTime)/1000.0;  // to seconds, or it is too big

        p = kP * current_error;
        i = i + kI * (current_error * delta_time);
        d = kD * (current_error - previousErr) / delta_time;

        RobotLogger.dd("PID i error: ", String.valueOf(i));
        RobotLogger.dd("PID current loc: ", String.valueOf(current) + " target loc: " + String.valueOf(target));

        if (i > MAX_I) {  // don't need too long history data
            i = MAX_I;
        }
        else if (i < -MAX_I) {
            i = -MAX_I;
        }

        double totalError  = p + i + d;

        previousErr = current_error;
        previousTime = currTime;
        RobotLogger.dd("PID update ", String.valueOf(p)+ " " + String.valueOf(i)+ " " + String.valueOf(d) + " Error: " + String.valueOf(totalError) +
                " curr_error: " + String.valueOf(current_error) +
                " delta time: " + String.valueOf(delta_time));

        return totalError;
    }
}