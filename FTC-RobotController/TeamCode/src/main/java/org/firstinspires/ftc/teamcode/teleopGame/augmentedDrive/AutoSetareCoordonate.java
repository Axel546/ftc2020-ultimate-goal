package org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.io.FileWriter;
import java.io.IOException;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Config
@TeleOp
public class AutoSetareCoordonate extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    SampleMecanumDrive drive;

    Boolean cheieScriere = Boolean.FALSE;
    Boolean isOk = Boolean.FALSE;
    Boolean lastTime = Boolean.FALSE;

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

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        drive.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        ElapsedTime telemetryTimer = new ElapsedTime();
        while (opModeIsActive()) {
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                        + VY_WEIGHT * Math.abs(baseVel.getY())
                        + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                        VX_WEIGHT * baseVel.getX(),
                        VY_WEIGHT * baseVel.getY(),
                        OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            drive.update();

            if (telemetryTimer.milliseconds() > 200) {
                telemetryTimer.reset();
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.update();
            }

            if(gamepad1.a && !cheieScriere)
            {
                isOk = !isOk;
                cheieScriere = !cheieScriere;
            }
            if(!gamepad1.a) {
                cheieScriere = false;
                isOk = false;
            }
            if(isOk && lastTime == Boolean.FALSE)
            {
                Pose2d poseEstimate = drive.getPoseEstimate();
                if(!((int) poseEstimate.getX() == (int) poseEstimate.getY() && (int) poseEstimate.getY() == (int) poseEstimate.getHeading() && (int) poseEstimate.getHeading() == 0)) {
                    try {
                        FileWriter myWriter = new FileWriter("coordinates.txt", true);
                        myWriter.write((int) poseEstimate.getX() + " " + (int) poseEstimate.getY() + " " + (int) poseEstimate.getHeading() + "\n");
                        myWriter.close();
                        System.out.println("Successfully wrote to the file.");
                        telemetry.addData("Scris txt", "da");
                        telemetry.update();
                    } catch (IOException e) {
                        System.out.println("An error occurred.");
                        telemetry.addData("Scris txt", "eroare");
                        telemetry.update();
                        e.printStackTrace();
                    }
                }
            }
            lastTime = isOk;

            intakeController();
            outtakeController();
            pistonController();
            servoBlockController();
            isPowerShotController();

            drive.outtakeMotor.setPower(outtakePower);
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
            outtakePower = 0.95;
        }
        else
            outtakePower = gamepad2.left_trigger - gamepad1.left_trigger;
    }
}