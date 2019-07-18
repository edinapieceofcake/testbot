package com.edinaftcrobotics.drivetrain;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.drivetrain.TelemetryMounts;
import com.edinaftcrobotics.navigation.RevImu;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Drive Assist", group="Teleop")

public class DriveAssist extends OpMode {
    private PieceOfCake robot;
    private Mecanum mecanum;
    private TelemetryMounts tm;
    private double rot;
    private imuChecker checker;
    private RevImu imu;

    @Override
    public void init() {

        robot = new PieceOfCake();
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);
        tm = new TelemetryMounts();

        try {
            imu = new RevImu(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        imu.start();

        checker = new imuChecker(imu);
        checker.start();

        rot = 0;


    }

    @Override
    public void loop() {

        rot = checker.getHeading();

        mecanum.assistedDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, rot);

        telemetry.addData("Rotation", rot);
        telemetry.update();

    }

    private class imuChecker extends Thread{

        private RevImu imu;
        private double rot;

        public imuChecker(RevImu imu){
            this.imu = imu;
        }

        public void run(){
            while(true){

                rot = imu.getAngle();

            }
        }

        public double getHeading(){
            return rot;
        }

    }
}
