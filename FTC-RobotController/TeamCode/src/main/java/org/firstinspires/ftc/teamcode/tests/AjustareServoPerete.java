package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.servo_perete;

@TeleOp
public class AjustareServoPerete extends LinearOpMode {
    servo_perete servoPerete = new servo_perete();
    Boolean anterior = Boolean.FALSE;

    @Override
    public void runOpMode() throws InterruptedException {
        servoPerete.initPerete(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a && !anterior) {
                servoPerete.open();
                anterior = true;
            }
            if(!gamepad1.a)
                anterior = false;
        }
    }
}
