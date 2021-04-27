package org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive.SampleMecanumDriveCancelable;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp
public class driveAuto extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector;
    // The heading we want the bot to end on for targetA
    double targetAHeading;

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(0);

    private List<Pose2d> allCoordinates;

    double minimum = 250000;

    Pose2d help;

    SampleMecanumDriveCancelable drive;

    double Scale = 1.0;
    double motorMax = 1.0;

    double intakePower = 0;
    double outtakePower = 0;
    double wobblePower = 0;
    double pistonPower = 0;
    double copieOutake = 0;

    Boolean boxIsUp = Boolean.FALSE;
    Boolean cheie = Boolean.FALSE;
    Boolean cheiePiston = Boolean.FALSE;
    Boolean isPistonOk = Boolean.FALSE;
    Boolean isRunning = Boolean.FALSE;
    Boolean cheieIntake = Boolean.FALSE;
    Boolean cheieOutake = Boolean.FALSE;
    Boolean isMax = Boolean.FALSE;
    Boolean precisionMode = Boolean.FALSE;
    Boolean cheiePrecision = Boolean.FALSE;
    Boolean isOpened = Boolean.FALSE;
    Boolean cheieWobbleS = Boolean.FALSE;
    Boolean isOpenedM = Boolean.FALSE;
    Boolean cheieWobbleM = Boolean.FALSE;
    Boolean isPowerShot = Boolean.FALSE;
    Boolean cheieOutakeP = Boolean.FALSE;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();

    Trajectory traj1;

    @Override
    public void runOpMode() {
        // Initialize custom cancelable SampleMecanumDrive class
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoWobble.initWobble(hardwareMap, true);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outake", outtakePower);
        telemetry.addData("Intake", intakePower);
        telemetry.update();

        drive.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        allCoordinates = new ArrayList<Pose2d>();

        /*File myObj = new File("filename.txt");

        Scanner scanner = null;
        scanner = new Scanner("coordinates.txt");
        while(scanner.hasNextDouble())
        {
            telemetry.addData("Ajuns",2);
            telemetry.update();
            allCoordinates.add(new Pose2d(scanner.nextDouble(),scanner.nextDouble(),scanner.nextDouble()));
        }

         */

        allCoordinates.add(new Pose2d(64.998,-4.638,Math.toRadians(0.5582)));
        allCoordinates.add(new Pose2d(65.227,-11.66,Math.toRadians(4.8322)));
        allCoordinates.add(new Pose2d(64.335,4.452,Math.toRadians(348.1343)));
        allCoordinates.add(new Pose2d(63.171,-20.739,Math.toRadians(2.7952)));
        allCoordinates.add(new Pose2d(61.658,6.226,Math.toRadians(352.2166)));
        allCoordinates.add(new Pose2d(63.103,-25.227,Math.toRadians(11.46333)));
        allCoordinates.add(new Pose2d(63.9675,16.872,Math.toRadians(345.9667)));
        allCoordinates.add(new Pose2d(64.6934,-2.4351,Math.toRadians(350.0764)));
        allCoordinates.add(new Pose2d(51.4533,-5.5392,Math.toRadians(351.5467)));
        allCoordinates.add(new Pose2d(51.1244,-14.3436,Math.toRadians(354.6095)));
        allCoordinates.add(new Pose2d(56.9545,10.0791,Math.toRadians(341.34222)));


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Array cu coord", allCoordinates);
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setDrivePower(
                            new Pose2d(
                                    -gamepad1.right_stick_y,
                                    -gamepad1.right_stick_x,
                                    -gamepad1.left_stick_x
                            )
                    );

                    if (gamepad1.y) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        minimum = 250000;

                        for(int i=0;i<allCoordinates.size();i++)
                        {
                            double m = Math.pow(poseEstimate.getX()- allCoordinates.get(i).getX(),2) + Math.pow(poseEstimate.getY()- allCoordinates.get(i).getY(),2);
                            if(m < minimum)
                            {
                                minimum = m;
                                targetAVector = new Vector2d(allCoordinates.get(i).getX(),allCoordinates.get(i).getY());
                                targetAHeading = Math.toRadians(allCoordinates.get(i).getHeading());
                            }
                        }

                        telemetry.addData("Going to",targetAHeading);
                        telemetry.addData("Heading",targetAHeading);
                        telemetry.update();

                        if(poseEstimate.getX()<64.5) {
                            traj1 = drive.trajectoryBuilder(poseEstimate)
                                    .splineTo(targetAVector, targetAHeading)
                                    .build();
                        }
                        else {
                            traj1 = drive.trajectoryBuilder(poseEstimate, true)
                                    .splineTo(targetAVector, targetAHeading+180)
                                    .build();
                        }

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.left_bumper) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            intakeController();
            outtakeController();
            pistonController();
            servoBlockController();
            servoWobbleController();
            wobbleMotorController();
            isPowerShotController();

            //giving power
            drive.intakeMotor.setPower(intakePower);
            drive.outtakeMotor.setPower(outtakePower);
            drive.wobbleMotor.setPower(wobblePower);
        }
    }

    void pistonController()
    {
        if(gamepad2.left_bumper && !cheiePiston)
        {
            pistonPower = 0.4;
            drive.pistonMotor.setPower(pistonPower);
            drive.pistonMotor.setTargetPosition(-170);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            drive.pistonMotor.setTargetPosition(0);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cheiePiston = true;
        }
        if(!gamepad2.left_bumper)
            cheiePiston = false;
    }

    void servoBlockController()
    {
        if(intakePower>0)
            servoBlock.close();
        else {
            if (gamepad2.a && !cheie) {
                boxIsUp = !boxIsUp;
                cheie = !cheie;
            }
            if (!gamepad2.a) {
                cheie = false;
            }
            if (boxIsUp)
                servoBlock.open();
            else
                servoBlock.close();
        }
    }

    void servoWobbleController()
    {
        if(gamepad1.a && !cheieWobbleS)
        {
            isOpened = !isOpened;
            cheieWobbleS = !cheieWobbleS;
        }
        if(!gamepad1.a)
            cheieWobbleS = false;
        if(isOpened)
            servoWobble.open();
        else
            servoWobble.close();
    }

    void wobbleMotorController()
    {
        if(gamepad1.b && !cheieWobbleM)
        {
            isOpenedM = !isOpenedM;
            cheieWobbleM = !cheieWobbleM;
        }
        if(!gamepad1.b)
            cheieWobbleM = false;
        if(isOpenedM) {
            wobblePower = 0.2;
            drive.wobbleMotor.setTargetPosition(700);
            drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            drive.wobbleMotor.setTargetPosition(0);
            drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobblePower = 0.2;
        }
    }

    void intakeController()
    {
        if(gamepad2.x && !cheieIntake)
        {
            isRunning = !isRunning;
            cheieIntake = !cheieIntake;
        }
        if(!gamepad2.x)
        {
            cheieIntake = false;
        }
        if(isRunning)
        {
            intakePower = 1.0;
        }
        else {
            if(gamepad2.right_trigger!=0)
            {
                intakePower = -gamepad2.right_trigger;
            }
            else
                intakePower = 0.0;
        }
    }

    void outtakeController()
    {
        if(gamepad2.y && !cheieOutake)
        {
            isMax = !isMax;
            cheieOutake = !cheieOutake;
        }
        if(!gamepad2.y)
            cheieOutake = false;
        if(isMax)
        {
            outtakePower = 1.0;
        }
        else
        {
            if(gamepad2.left_trigger!=0)
            {
                outtakePower = gamepad2.left_trigger;
            }
            else
                outtakePower = 0.0;
        }
    }

    void isPowerShotController()
    {
        if(gamepad2.b && !cheieOutakeP)
        {
            isPowerShot = !isPowerShot;
            cheieOutakeP = !cheieOutakeP;
        }
        if(!gamepad2.b)
            cheieOutakeP = false;
        if(isPowerShot)
        {
            outtakePower = 0.9;
        }
        else
            outtakePower = gamepad2.left_trigger - gamepad1.left_trigger;
    }
}