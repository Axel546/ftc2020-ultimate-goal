package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive.PoseStorage;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class AutoRegionala extends LinearOpMode {

    public SampleMecanumDrive bot;
    public double powershotPower = 0.92;
    public double blocPos = 0.17;

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
    servo_perete servoPerete = new servo_perete();

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, false);
        servoPerete.initPerete(hardwareMap);

        initVuforia();
        initTfod();

        if(tfod != null)
        {
            tfod.activate();
        }

        while(!isStarted())
        {
            if (tfod != null) {
                tfod.setClippingMargins(5,5,5,5);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("rings", "0");
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                zona = 1.0;
                                telemetry.addData("rings", "1");
                            } else if (recognition.getLabel().equals("Quad")) {
                                zona = 4.0;
                                telemetry.addData("rings", "4");
                            } else {
                                zona = 0.0;
                            }
                        }
                    }
                }
            }
        }

        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tfod.shutdown();

        if (isStopRequested()) return;

        if(zona == 0.0) {
            Trajectory trajPowershot1 = bot.trajectoryBuilder(new Pose2d())
                    .addTemporalMarker(0,()->{
                        bot.outtakeMotor.setPower(powershotPower);
                    })
                    .splineTo(new Vector2d(63, 27), Math.toRadians(3))
                    .build();

            Trajectory trajPowershot2 = bot.trajectoryBuilder(trajPowershot1.end())
                    .strafeTo(new Vector2d(63, 19))
                    .build();

            Trajectory trajPowershot3 = bot.trajectoryBuilder(trajPowershot2.end())
                    .strafeTo(new Vector2d(63, 10))
                    .build();

            Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot3.end())
                    .splineTo(new Vector2d(69, -18), Math.toRadians(-90))
                    .addTemporalMarker(0, () -> {
                        bot.outtakeMotor.setPower(0);
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.15);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        servoBlock.close();
                    })
                    .build();

            Trajectory littleBack = bot.trajectoryBuilder(putAwayWobble1.end())
                    .strafeTo(new Vector2d(69, -8))
                    .build();

            Trajectory returnToBase = bot.trajectoryBuilder(littleBack.end())
                    .splineTo(new Vector2d(45, -15), Math.toRadians(180))
                    .build();

            Trajectory littleFront = bot.trajectoryBuilder(returnToBase.end())
                    .strafeTo(new Vector2d(30, -15))
                    .build();

            Trajectory putAwayWobble2 = bot.trajectoryBuilder(littleFront.end().plus(new Pose2d(0, 0, Math.toRadians(-180))))
                    .splineTo(new Vector2d(63, -22.5), 0)
                    .addTemporalMarker(0,()->{
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.1);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .build();

            Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                    .back(3)
                    .splineToConstantHeading(new Vector2d(65, 0), 0)
                    .build();


            bot.followTrajectory(trajPowershot1);
            sleep(100);
            shoot(1, false);
            bot.turn(Math.toRadians(-2));
            bot.followTrajectory(trajPowershot2);
            shoot(1, true);
            bot.followTrajectory(trajPowershot3);
            shoot(1, false);
            bot.followTrajectory(putAwayWobble1);
            servoWobble.close();
            sleep(100);

            bot.followTrajectory(littleBack);

            bot.followTrajectory(returnToBase);

            bot.followTrajectory(littleFront);
            servoWobble.open();
            sleep(500);
            bot.wobbleMotor.setTargetPosition(-300);
            bot.wobbleMotor.setPower(0.3);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            bot.turn(Math.toRadians(180));

            bot.followTrajectory(putAwayWobble2);

            servoWobble.close();
            sleep(100);
            bot.followTrajectory(parkRobot);
        }
        else if(zona == 1.0) {
            Trajectory trajPowershot1 = bot.trajectoryBuilder(new Pose2d())
                    .addTemporalMarker(0, () -> {
                        bot.outtakeMotor.setPower(powershotPower);
                    })
                    .splineTo(new Vector2d(63, 27), Math.toRadians(2))
                    .build();

            Trajectory trajPowershot2 = bot.trajectoryBuilder(trajPowershot1.end())
                    .strafeTo(new Vector2d(63,19))
                    .build();

            Trajectory trajPowershot3 = bot.trajectoryBuilder(trajPowershot2.end())
                    .strafeTo(new Vector2d(63,10))
                    .build();

            Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot3.end())
                    .addTemporalMarker(0, () -> {
                        bot.outtakeMotor.setPower(0);
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.17);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        servoBlock.close();
                    })
                    .splineTo(new Vector2d(80,2),0)
                    .build();

            Trajectory littleBack = bot.trajectoryBuilder(putAwayWobble1.end())
                    .strafeTo(new Vector2d(60,4))
                    .build();

            Trajectory takeRing = bot.trajectoryBuilder(littleBack.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                    .addTemporalMarker(0,()->{
                        bot.intakeMotor.setPower(1);
                    })
                    .strafeTo(new Vector2d(50,-4))
                    .build();

            Trajectory returnToBase = bot.trajectoryBuilder(takeRing.end())
                    .splineToConstantHeading(new Vector2d(45,-16.5),Math.toRadians(180))
                    .build();

            Trajectory littleFront = bot.trajectoryBuilder(returnToBase.end())
                    .strafeTo(new Vector2d(32,-16.5))
                    .build();

            Trajectory trajShoot = bot.trajectoryBuilder(littleFront.end().plus(new Pose2d(0, 0, Math.toRadians(-180))))
                    .addTemporalMarker(0,()->{
                        bot.outtakeMotor.setPower(1);
                        servoBlock.open();
                    })
                    .splineTo(new Vector2d(63,-4),Math.toRadians(0))
                    .build();

            Trajectory putAwayWobble2 = bot.trajectoryBuilder(trajShoot.end())
                    .splineTo(new Vector2d(80,-4),0)
                    .addTemporalMarker(0,()->{
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.1);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .build();


            Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                    .strafeTo(new Vector2d(65,0))
                    .build();


            bot.followTrajectory(trajPowershot1);
            sleep(100);
            shoot(1, false);
            bot.turn(Math.toRadians(-2));
            bot.followTrajectory(trajPowershot2);
            sleep(100);
            shoot(1, true);
            bot.followTrajectory(trajPowershot3);
            sleep(100);
            shoot(1, false);
            bot.followTrajectory(putAwayWobble1);
            servoWobble.close();
            sleep(100);

            bot.followTrajectory(littleBack);

            bot.turn(Math.toRadians(180));

            bot.followTrajectory(takeRing);

            bot.followTrajectory(returnToBase);

            bot.followTrajectory(littleFront);
            servoWobble.open();
            sleep(600);
            bot.wobbleMotor.setTargetPosition(-300);
            bot.wobbleMotor.setPower(0.3);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            bot.turn(Math.toRadians(180));

            bot.followTrajectory(trajShoot);
            shoot(1,false);
            sleep(200);
            bot.outtakeMotor.setPower(0);
            bot.intakeMotor.setPower(0);
            servoBlock.close();

            bot.followTrajectory(putAwayWobble2);
            servoWobble.close();
            sleep(100);
            bot.followTrajectory(parkRobot);

        }
        else
        {
            Trajectory trajPowershot = bot.trajectoryBuilder(new Pose2d())
                    .addTemporalMarker(0, ()->{
                        bot.outtakeMotor.setPower(powershotPower);
                    })
                    .splineTo(new Vector2d(63, 27),Math.toRadians(3))
                    .build();

            Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot.end())
                    .addTemporalMarker(0,()->{
                        bot.outtakeMotor.setPower(0);
                        bot.wobbleMotor.setTargetPosition(-700);
                        bot.wobbleMotor.setPower(-0.2);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .addSpatialMarker(new Vector2d(111,-4),()->{
                        servoWobble.close();
                    })
                    .splineTo(new Vector2d(112,-6),Math.toRadians(-90))
                    .addDisplacementMarker(()->{
                        servoWobble.close();
                        sleep(100);
                    })
                    .splineTo(new Vector2d(90,-16),Math.toRadians(180))
                    .addDisplacementMarker(()->{
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.2);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    })
                    .splineTo(new Vector2d(53,-15),Math.toRadians(145))
                    .addDisplacementMarker(()->{
                        bot.intakeMotor.setPower(1);
                        servoBlock.close();
                    })
                    .build();

            Trajectory takeRings = bot.trajectoryBuilder(putAwayWobble1.end())
                    /*.addTemporalMarker(0.35,()->{
                        servoPerete.open();
                    })
                    .addTemporalMarker(0.55,()->{
                        servoPerete.close();
                    })

                     */
                    .strafeTo(new Vector2d(32.5,3)/*,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)*/
                    )
                    .build();

            Trajectory takeWobble = bot.trajectoryBuilder(putAwayWobble1.end())
                    .splineTo(new Vector2d(40,-16.5),Math.toRadians(185))
                    .build();

            Trajectory littleFront = bot.trajectoryBuilder(takeWobble.end())
                    .strafeTo(new Vector2d(29,-16.5))
                    .addDisplacementMarker(()->{
                        servoWobble.open();
                        sleep(200);
                    })
                    .build();

            Trajectory trajShoot = bot.trajectoryBuilder(takeWobble.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                    .addTemporalMarker(0,()->{
                        bot.outtakeMotor.setPower(1);
                    })
                    .splineTo(new Vector2d(63,-4),Math.toRadians(3))
                    .addDisplacementMarker(()->{
                        servoBlock.open();
                    })
                    .build();

            Trajectory putAwayWobble2 = bot.trajectoryBuilder(trajShoot.end())
                    .addTemporalMarker(0,()->{
                        bot.wobbleMotor.setTargetPosition(-900);
                        bot.wobbleMotor.setPower(-0.15);
                        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        bot.outtakeMotor.setPower(0);
                        bot.intakeMotor.setPower(0);
                    })
                    .addSpatialMarker(new Vector2d(118,-18),()->{
                        servoWobble.close();
                    })
                    .splineTo(new Vector2d(100,-20),0)
                    .build();

            Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                    .strafeTo(new Vector2d(65,0))
                    .build();

            bot.followTrajectory(trajPowershot);
            shoot(1,false);
            bot.turn(Math.toRadians(-9));
            shoot(1,false);
            bot.turn(Math.toRadians(-9));
            shoot(1, false);
            bot.followTrajectory(putAwayWobble1);
            bot.followTrajectory(takeRings);
            bot.followTrajectory(takeWobble);
            bot.followTrajectory(littleFront);
            sleep(300);
            bot.wobbleMotor.setTargetPosition(-300);
            bot.wobbleMotor.setPower(0.3);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.turn(Math.toRadians(180));
            bot.followTrajectory(trajShoot);
            sleep(150);
            shoot(3,false);
            bot.followTrajectory(putAwayWobble2);
            bot.followTrajectory(parkRobot);
        }
        PoseStorage.currentPose = bot.getPoseEstimate();

        stop();

    }
    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            bot.pistonMotor.setPower(0.3);
            bot.pistonMotor.setTargetPosition(-165);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(380);
            bot.pistonMotor.setTargetPosition(0);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(380);

            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(200);
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}