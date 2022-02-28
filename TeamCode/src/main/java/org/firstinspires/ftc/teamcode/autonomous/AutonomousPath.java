package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Teleop.TeleopConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MyHardwareMap;
import org.firstinspires.ftc.teamcode.util.MyHardwareMap;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.VuforiaCamLocalizer;

import java.util.List;

import static java.lang.Math.PI;

@Disabled
public class AutonomousPath {
    private Pose2d startingPos;
    private SampleMecanumDrive drive; // default drive;
    private int step_count = 0;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    //private Align align;
    private MyHardwareMap hwMap;
    private LinearOpMode opMode;
    private List<Recognition> tfod;
    private static String TAG = "AutonomousPath";
    private Pose2d currentPos;
    private Telemetry telemetry;
    private String path_file;
    private int first_skystone_location = 0;

    public AutonomousPath(LinearOpMode opMode, SampleMecanumDrive drive,
                          com.qualcomm.robotcore.hardware.HardwareMap hwMap, BNO055IMU imu, Telemetry telemetry) {
        //this.startingPos = startingPos;
        this.opMode = opMode;
        this.hwMap = new MyHardwareMap(hwMap);

        this.drive = drive;
        this.telemetry = telemetry;
        //vu = new VuforiaCamLocalizer(hardwareMap);
    }

    public int RunPath(Pose2d coordinates[], String ops[], VuforiaCamLocalizer vLocal) {
        for (int step_count = 0; step_count < coordinates.length; step_count ++) {
            Pose2d currLocation = drive.getPoseEstimate();
            builder = drive.trajectoryBuilder(currLocation);

            RobotLogger.dd(TAG, "current pose: " + currLocation.toString());
            RobotLogger.dd(TAG, "step" + Integer.toString(step_count) + coordinates[step_count].toString() + ' ' + ops[step_count]);
            Trajectory traj = null;
            if (ops[step_count].equals("splineTo"))
                traj = builder.splineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()), 0).build();
            else if (ops[step_count].equals("lineTo"))
                traj = builder.lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY())).build();
            else if (ops[step_count].equals("strafeTo"))
                traj = builder.strafeTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY())).build();
            else
                assert(true);
            drive.followTrajectory(traj);
            SafeSleep.sleep_milliseconds(opMode, 400);
        }
        return 0;
    }
}
//                "after straight move, grabbed 2nd, to straight move");
//        builder = builder
//                .lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
//        trajectory = builder.build();   //x - 2.812, y + 7.984
//        if (opMode.opModeIsActive())
//            _drive.followTrajectory(trajectory);
//        step_count++;
//
//        if (vLocal != null) {
//            Pose2d t = vLocal.getPoseEstimate();
//            RobotLogger.dd(TAG, "Calibrate before drop 2nd stone! Vuforia local info: " + t.toString());
//        }
//        RobotLogger.dd(TAG, "step4.5, after straight move, to drop");
//
//        SafeSleep.sleep_milliseconds(opMode, 400);
