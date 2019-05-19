package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: Encoder Position", group = "Test")
@Disabled
public class EncoderPositionTest extends LinearOpMode {
    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getFrontL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Flip Location", "%d", robot.getFrontFlip().getCurrentPosition());
            telemetry.addData("Slide Location", "%d", robot.getSlide().getCurrentPosition());
            telemetry.addData("Back Lift Location", "%d", robot.getBackLift().getCurrentPosition());
            telemetry.addData("Front Lift Location", "%d", robot.getFrontLift().getCurrentPosition());

            telemetry.addData("Front Left Location", "%d", robot.getFrontL().getCurrentPosition());
            telemetry.addData("Front Right Location", "%d", robot.getFrontR().getCurrentPosition());
            telemetry.addData("Back Left Location", "%d", robot.getBackL().getCurrentPosition());
            telemetry.addData("Back Right Location", "%d", robot.getBackR().getCurrentPosition());
            telemetry.update();
        }
    }
}
