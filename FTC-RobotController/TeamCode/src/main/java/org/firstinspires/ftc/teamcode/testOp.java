package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class testOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Encoder leftEncoder, rightEncoder, frontEncoder;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "outtakeMotor"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeMotor"));

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("y", frontEncoder.getCurrentPosition());
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}