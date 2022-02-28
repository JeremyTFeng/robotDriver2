package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;
import java.util.List;

public class MyHardwareMap {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors;

    public BNO055IMU imu;
    public Servo clawServo1, clawServo2, foundationLock, transferLock, transferHorn,
            clawInit, innerTransfer, parkingServo,
            redAutoClawJoint1, redAutoClawJoint2, redAutoClawJoint3,
            liftOdometry;
    public DigitalChannel liftReset, foundationDetectLeft, foundationDetectRight;
    HardwareMap hardwareMap;
    public MyHardwareMap(HardwareMap hwMap) {
        hardwareMap = hwMap;
        if (!DriveConstants.VirtualizeDrive)
            loadHardwareFromConfig();
    }
    private void loadHardwareFromConfig() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
/*
        //region
        //------------------------===Servos===------------------------

        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
        foundationLock = hardwareMap.get(Servo.class, "foundationLock");
        redAutoClawJoint1 = hardwareMap.get(Servo.class, "redAutoClawJoint1");
        redAutoClawJoint2 = hardwareMap.get(Servo.class, "redAutoClawJoint2");
        redAutoClawJoint3 = hardwareMap.get(Servo.class, "redAutoClawJoint3");
        transferLock = hardwareMap.get(Servo.class, "transferLock");
        transferHorn = hardwareMap.get(Servo.class, "transferHorn");
        clawInit = hardwareMap.get(Servo.class, "clawInit");
        innerTransfer = hardwareMap.get(Servo.class, "innerTransfer");
        parkingServo = hardwareMap.get(Servo.class, "parkingServo");
        //parkingServo = hwMap.get(Servo.class, "parkingServo");  //@TODO Configure intakeInit on the robot
        liftOdometry = hardwareMap.get(Servo.class, "liftOdometry");

        //---------------------------------------------------------------------------
        //endregion

        liftReset = hardwareMap.get(DigitalChannel.class, "liftReset");
        //intakeDetect = hwMap.get(DigitalChannel.class, "intakeDetect");   //@TODO Configure intakeDetect on the robot
        foundationDetectLeft = hardwareMap.get(DigitalChannel.class, "foundationDetectLeft");
        foundationDetectRight = hardwareMap.get(DigitalChannel.class, "foundationDetectRight");
    }
*/
    }
}
