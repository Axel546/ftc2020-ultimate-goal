package org.firstinspires.ftc.teamcode.tests;

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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.util.Arrays;

@Autonomous
public class TestAuto4 extends LinearOpMode {

    SampleMecanumDrive drive;

    servo_wobble servoWobble = new servo_wobble();
    servo_block servoBlock = new servo_block();
    servo_perete servoPerete = new servo_perete();

    double powershotPower = 0.95;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        servoPerete.initPerete(hardwareMap);
        servoWobble.initWobble(hardwareMap,false);
        servoBlock.initBlock(hardwareMap);
        servoPerete.initPerete(hardwareMap);
        servoBlock.open();

        Trajectory trajPowershot = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, ()->{
                    drive.outtakeMotor.setPower(powershotPower);
                })
                .splineTo(new Vector2d(63, 27),Math.toRadians(0))

                .build();

        Trajectory putAwayWobble1 = drive.trajectoryBuilder(trajPowershot.end())
                .addTemporalMarker(0,()->{
                    drive.outtakeMotor.setPower(0);
                    drive.wobbleMotor.setTargetPosition(-700);
                    drive.wobbleMotor.setPower(-0.2);
                    drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addSpatialMarker(new Vector2d(111,0),()->{
                    servoWobble.close();
                })
                .splineTo(new Vector2d(112,-6),Math.toRadians(-90))
                .addDisplacementMarker(()->{
                    servoWobble.close();
                    sleep(100);
                })
                .splineTo(new Vector2d(90,-16),Math.toRadians(180))
                .addDisplacementMarker(()->{
                    drive.wobbleMotor.setTargetPosition(-900);
                    drive.wobbleMotor.setPower(-0.2);
                    drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .splineTo(new Vector2d(53,-15),Math.toRadians(145))
                .addDisplacementMarker(()->{
                    drive.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory takeRings = drive.trajectoryBuilder(putAwayWobble1.end())
                /*.addTemporalMarker(0.35,()->{
                    servoPerete.open();
                })
                .addTemporalMarker(0.55,()->{
                    servoPerete.close();
                })

                 */
                .strafeTo(new Vector2d(30,3)/*,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)*/
                )
                .build();

        Trajectory takeWobble = drive.trajectoryBuilder(putAwayWobble1.end())
                .splineTo(new Vector2d(34,-17),Math.toRadians(185))
                .addDisplacementMarker(()->{
                    sleep(300);
                    servoWobble.open();
                })
                .build();

        Trajectory trajShoot = drive.trajectoryBuilder(takeWobble.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .addTemporalMarker(0,()->{
                    drive.outtakeMotor.setPower(1);
                })
                .splineTo(new Vector2d(63,-4),Math.toRadians(5))
                .addDisplacementMarker(()->{
                    servoBlock.open();
                })
                .build();

        Trajectory putAwayWobble2 = drive.trajectoryBuilder(trajShoot.end())
                .addTemporalMarker(0,()->{
                    drive.wobbleMotor.setTargetPosition(-900);
                    drive.wobbleMotor.setPower(-0.15);
                    drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.outtakeMotor.setPower(0);
                    drive.intakeMotor.setPower(0);
                })
                .addSpatialMarker(new Vector2d(117,-18),()->{
                    servoWobble.close();
                })
                .splineTo(new Vector2d(100,-20),0)
                .build();

        Trajectory parkRobot = drive.trajectoryBuilder(putAwayWobble2.end())
                .strafeTo(new Vector2d(65,0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Pose2d(30, 30, 0))
//                .build();



        /*Trajectory takeRings = new TrajectoryBuilder(putAwayWobble1.end().plus(new Pose2d(0,0,Math.toRadians(5))), drive.constraints)
                .splineTo(new Vector2d(48,0),Math.toRadians(100))
                .splineTo(new Vector2d(38,-11),Math.toRadians(180))
                .build();
         */

        /*Trajectory littleBack = new TrajectoryBuilder(putAwayWobble1.end(), drive.constraints)
                .strafeTo(new Vector2d(69,-8))
                .build();
        Trajectory returnToBase = new TrajectoryBuilder(littleBack.end(), drive.constraints)
                .splineTo(new Vector2d(45,-11),Math.toRadians(180))
                .build();
        Trajectory littleFront = new TrajectoryBuilder(returnToBase.end(), drive.constraints)
                .strafeTo(new Vector2d(30,-11))
                .build();
        Trajectory putAwayWobble2 = new TrajectoryBuilder(littleFront.end().plus(new Pose2d(0,0,Math.toRadians(-180))), drive.constraints)
                .splineTo(new Vector2d(63,-22.5),0)
                .build();
        Trajectory trajShoot = new TrajectoryBuilder(putAwayWobble2.end(), drive.constraints)
                .back(3)
                .splineToConstantHeading(new Vector2d(63,-8),0)
                .build();
        Trajectory parkRobot = new TrajectoryBuilder(trajShoot.end(), drive.constraints)
                .strafeTo(new Vector2d(65,-8))
                .build();
         */


        drive.followTrajectory(trajPowershot);
        shoot(1,false);
        drive.turn(Math.toRadians(-8));
        shoot(1,true);
        drive.turn(Math.toRadians(-8));
        shoot(1, false);
        drive.followTrajectory(putAwayWobble1);
        drive.followTrajectory(takeRings);
        drive.followTrajectory(takeWobble);
        sleep(250);
        drive.wobbleMotor.setTargetPosition(-300);
        drive.wobbleMotor.setPower(0.3);
        drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(trajShoot);
        sleep(150);
        shoot(3,true);
        drive.followTrajectory(putAwayWobble2);
        drive.followTrajectory(parkRobot);
        //drive.followTrajectory(takeRings);
        //drive.turn(Math.toRadians(-10));

        /*drive.followTrajectory(littleBack);
        drive.followTrajectory(returnToBase);
        drive.followTrajectory(littleFront);
        sleep(500);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(putAwayWobble2);
        sleep(500);
        drive.followTrajectory(trajShoot);
        sleep(500);
        drive.followTrajectory(parkRobot);
         */

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(0)))
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            drive.pistonMotor.setPower(0.3);
            drive.pistonMotor.setTargetPosition(-165);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(400);
            drive.pistonMotor.setTargetPosition(0);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(400);

            if(boxDown)
            {
                servoBlock.close();
                sleep(200);
                servoBlock.open();
                sleep(200);
            }
        }
    }
}
