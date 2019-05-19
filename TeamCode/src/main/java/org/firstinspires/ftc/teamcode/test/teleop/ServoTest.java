package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test: Servo", group = "Teleop Test")
@Disabled
public class ServoTest extends LinearOpMode {
    public void runOpMode() {

        ServoImplEx ts = hardwareMap.get(ServoImplEx.class, "ts");

        ts.setPwmRange(new PwmControl.PwmRange(400, 2700));

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                ts.setPosition(1);
            } else if (gamepad1.b){
                ts.setPosition(-1);
            }
            else {
                ts.setPosition(0);
            }
        }

    }
}
