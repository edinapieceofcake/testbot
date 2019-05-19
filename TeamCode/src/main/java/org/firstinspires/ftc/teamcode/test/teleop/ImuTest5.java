package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.TurnOMatic;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "Test: IMU5", group = "Teleop Test")
@Disabled
public class ImuTest5 extends LinearOpMode {
    BNO055IMU imu = null;
    Mecanum mecanum = null;

    public void runOpMode() {
        long counter = 0;
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        robot.getFrontL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);

        SetupIMU();

        telemetry.addData("Waiting", "for start");
        telemetry.update();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addData("Current Angle", "%f", GetImuAngle());
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        //mecanum.TurnLeft(.5, 1200, this);

        telemetry.addData("Starting", "now");
        telemetry.update();

        TurnOMatic turner = new TurnOMatic(imu, mecanum, telemetry, 90, this, .7);
        turner.Turn(.05, 3000);
            //mecanum.Move(-.0, 0, -.3, .3);

        mecanum.MoveForward2(.5, 500, this);

        robot.getFrontL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnOMatic turner2 = new TurnOMatic(imu, mecanum, telemetry, 135, this, .7);
        turner2.Turn(.05, 3500);

        mecanum.MoveForward2(.5, 500, this);

        robot.getFrontL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurnOMatic turner3 = new TurnOMatic(imu, mecanum, telemetry, -45, this, 1);
        turner3.Turn(.05, 3000);

        //mecanum.MoveForward(.5, 500, this);

        sleep(30000);
    }

    private void SetupIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
}
