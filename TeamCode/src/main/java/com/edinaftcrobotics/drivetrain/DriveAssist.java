package com.edinaftcrobotics.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Drive Assist", group="Teleop")

public class DriveAssist extends OpMode {
    private PieceOfCake robot;
    private Mecanum mecanum;
    private TelemetryMounts tm;
    private int oldl, olds, oldr;

    @Override
    public void init() {

        robot = new PieceOfCake();
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);
        tm = new TelemetryMounts(2, 4, 1400, 15.5);
        oldl = 0;
        oldr = 0;
        olds = 0;

        mecanum.setCurrentPower(0.5);

    }

    @Override
    public void loop() {

        if(gamepad1.a){
            mecanum.StopAndResetMotors3();
            tm.set(0,0,0);
        }

        int leftPos = robot.getBackL().getCurrentPosition();
        int strafePos = robot.getFrontR().getCurrentPosition();
        int rightPos = robot.getFrontL().getCurrentPosition();

//        Taking the change in the encoder values
        int left = leftPos - oldl;
        int strafe = strafePos - olds;
        int right = rightPos - oldr;

//        Setting old positions to be used in the next iteration
        oldl = leftPos;
        olds = strafePos;
        oldr = rightPos;

//        This function needs to be called with the change of encoder values every iteration of the main loop
        tm.update(-right, left, strafe);

//        This calls the localized driving system that takes in the heading value from the telemetry mounts
        mecanum.assistedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x * 0.5, 360 - tm.getHeading());

        telemetry.addData("Rotation", tm.getHeading() + ", X: " + tm.getX() + ", Y: " + tm.getY());
        telemetry.addData("Translation", "left: " + left + ", strafe: " + strafe + ", right :" + right);
        telemetry.addData("Position", "left: " + leftPos + ", strafe: " + strafePos + ", right :" + rightPos);
        telemetry.update();


    }

}
