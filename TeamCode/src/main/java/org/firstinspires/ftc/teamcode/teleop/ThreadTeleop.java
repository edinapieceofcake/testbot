package org.firstinspires.ftc.teamcode.teleop;

import com.edinaftcrobotics.localization.Vector2d;
import com.edinaftcrobotics.subsystems.MecanumDrive;
import com.edinaftcrobotics.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ThreadTeleOp", group = "teleop")
public class ThreadTeleop extends OpMode {
    private StickyGamepad stickyGamepad1;

    private Robot robot;

    private boolean slowMode, superSlowMode;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        stickyGamepad1.update();

        // drive
        if (stickyGamepad1.b) {
            slowMode = !slowMode;
            superSlowMode = false;
            if (slowMode) {
                robot.drive.setVelocityPIDCoefficients(MecanumDrive.SLOW_VELOCITY_PID);
            } else {
                robot.drive.setVelocityPIDCoefficients(MecanumDrive.NORMAL_VELOCITY_PID);
            }
        } else if (stickyGamepad1.left_bumper) {
            superSlowMode = !superSlowMode;
            slowMode = false;
            if (superSlowMode) {
                robot.drive.setVelocityPIDCoefficients(MecanumDrive.SLOW_VELOCITY_PID);
            } else {
                robot.drive.setVelocityPIDCoefficients(MecanumDrive.NORMAL_VELOCITY_PID);
            }
        }

        double x, y = 0, omega;

        x = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        omega = -gamepad1.right_stick_x;

        if (superSlowMode) {
            x *= 0.25;
            y *= 0.25;
            omega *= 0.25;
        } else if (slowMode) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        robot.drive.setVelocity(new Vector2d(x, y), omega);

    }
}

