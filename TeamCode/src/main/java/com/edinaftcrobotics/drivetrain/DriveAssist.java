package com.edinaftcrobotics.drivetrain;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.drivetrain.TelemetryMounts;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Drive Assist", group="Teleop")

public class DriveAssist extends OpMode {
    private PieceOfCake robot;
    private Mecanum mecanum;
    private TelemetryMounts tm;

    @Override
    public void init() {

        robot = new PieceOfCake();
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);
        tm = new TelemetryMounts(2.875, 4, 360, 15.5);

    }

    @Override
    public void loop() {
        mecanum.StopAndResetMotors3();
        int left = robot.getBackR().getCurrentPosition();
        int strafe = robot.getBackL().getCurrentPosition();
        int right = robot.getFrontR().getCurrentPosition();

        tm.update(right, left, strafe);

        mecanum.assistedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, tm.getHeading());
        telemetry.addData("Rotation", tm.getHeading());
        telemetry.update();


    }

}
