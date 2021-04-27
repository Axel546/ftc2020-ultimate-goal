package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

@Autonomous
public class AutoTest extends LinearOpMode {

    public SampleMecanumDrive bot;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //developer key
    private static final String VUFORIA_KEY =
            "AfEZkiL/////AAABmaamsSiv9Ekeqgkncg9w6UKHCKU6c+yfO47f26KKNNRR5bu3Dqtk1794PqGUq3NuWygJPWjhsUkSBbDXsvzWfpHhGvZPx+TII7Io4o7hB8uAul0lxS1eywdu1374gI74XkwUD3rp08eW1EGW8nSrMXrwQgpx4ivWP7eVAFRGtciEYZ6+XC/tK0R9csWhRalAw1fcgFHJDFeO6NK3xgk01vVAItO3GRXjzEim9um6iWC70s67xCRFM+s2j+0oVCVyop5aPZ71Sn7k6wcSGW+eAgVtfNRslSBhEkUXaH0ThS6QaCeNhC4F44kuMpwMlrIwxZiiHJX1muex8zfcGN8alM+b67q8sJ8kO+QY5ePgMkxQ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public double zona = 0;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();

    @Override
    public void runOpMode() {
        bot = new SampleMecanumDrive(hardwareMap);
        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, false);


        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Trajectory trajPowershot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0.2, () -> {
                    bot.outtakeMotor.setPower(0.95);
                })
                .splineTo(new Vector2d(63, 27), 0)
                .build();
        Trajectory trajPowershot2 = bot.trajectoryBuilder(trajPowershot.end())
                .strafeTo(new Vector2d(63, 18))
                .addTemporalMarker(0, () -> {
                    servoBlock.close();
                    servoBlock.open();
                })
                .build();
        Trajectory trajPowershot3 = bot.trajectoryBuilder(trajPowershot2.end())
                .strafeTo(new Vector2d(63, 11))
                .addTemporalMarker(0, () -> {
                    servoBlock.close();
                    servoBlock.open();
                })
                .build();
        Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot3.end())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setPower(0);
                    bot.wobbleMotor.setTargetPosition(-900);
                    bot.wobbleMotor.setPower(-0.15);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(72, -15), -Math.toRadians(90))
                .build();
        Trajectory returnToBase = bot.trajectoryBuilder(putAwayWobble1.end(), true)
                .splineTo(new Vector2d(45, -15), 0)
                .build();
        Trajectory takeWobble2 = bot.trajectoryBuilder(returnToBase.end())
                .strafeTo(new Vector2d(30, -15))
                .build();
        Trajectory putAwayWobble2 = bot.trajectoryBuilder(takeWobble2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .addTemporalMarker(1, () -> {
                    bot.wobbleMotor.setTargetPosition(-900);
                    bot.wobbleMotor.setPower(-0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(60, -23), 0)
                .build();
        Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                .back(3)
                .splineToConstantHeading(new Vector2d(63, -7), 0)
                .build();
        bot.followTrajectory(trajPowershot);
        sleep(100);
        shoot(1, false);
        bot.followTrajectory(trajPowershot2);
        shoot(1, true);
        bot.followTrajectory(trajPowershot3);
        shoot(1, false);
        bot.followTrajectory(putAwayWobble1);
        servoWobble.close();
        sleep(100);

        bot.followTrajectory(returnToBase);

        bot.followTrajectory(takeWobble2);
        servoWobble.open();
        sleep(600);
        bot.wobbleMotor.setTargetPosition(-300);
        bot.wobbleMotor.setPower(0.3);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bot.turn(Math.toRadians(180));

        bot.followTrajectory(putAwayWobble2);

        servoWobble.close();
        sleep(200);
        bot.followTrajectory(parkRobot);
        stop();
    }

    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            bot.pistonMotor.setPower(0.3);
            bot.pistonMotor.setTargetPosition(-160);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            bot.pistonMotor.setTargetPosition(0);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);

            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(100);
            }
        }
    }
}