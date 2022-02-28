package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousPath;
import org.firstinspires.ftc.teamcode.autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MyHardwareMap;
import org.firstinspires.ftc.teamcode.util.ParseXYFromXML;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.vision.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.vision.VuforiaCameraChoice;


@Config
@Autonomous(group = "drive")
public class AutonomousPathTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "PathTest";
    private SampleMecanumDrive _drive = null, _strafeDrive = null;
    private AutonomousPath path;
    private FieldPosition fieldPosition = null;
    private VuforiaCamLocalizer vuLocalizer = null;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotLogger.dd(TAG, "xml file %d", 1);
        String filename = "path_blue_.xml";
        int tindex = filename.indexOf(".xml");
        RobotLogger.dd(TAG, "%d", tindex);
        filename = filename.substring(0, tindex-1) + Integer.toString(1)
                + filename.substring(tindex);
        ParseXYFromXML myParser = new ParseXYFromXML(filename);
        Pose2d coordinates[] = myParser.getCoordinates();
        String ops[] = myParser.getOperations();
        RobotLogger.dd(TAG, "XY array len: " + Integer.toString(coordinates.length));

        _drive = new SampleMecanumDrive(hardwareMap, this);

        fieldPosition = FieldPosition.BLUE_QUARY;

        waitForStart();

        if (isStopRequested()) return;

        if (DriveConstants.USE_VUFORIA_LOCALIZER) {
            vuLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap,
                    VuforiaCameraChoice.PHONE_BACK, true);
        }
        path = new AutonomousPath(this, _drive, hardwareMap, null, telemetry);
        path.RunPath(coordinates,  ops, null);
        RobotLogger.dd(TAG, "----------done --------------------- unit test for path (BLUE QUARY)");
    }
}
