package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: Mecanum", group = "Teleop Test")
@Disabled
public class MecanumTest extends LinearOpMode {
    private Mecanum _mecanum;
    protected BNO055IMU imu = null;

    public MecanumTest() {
    }

    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);

        _mecanum.StopAndResetMotors();
        InitGyro();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                _mecanum.SlideLeft2(.5, 350, this);
            } else if (gamepad1.dpad_right) {
                _mecanum.SlideRight2(.5, 1000, this);
            } else if (gamepad1.dpad_up) {
                _mecanum.MoveForward(.5, 350, this);
            } else if (gamepad1.dpad_down) {
                _mecanum.MoveBackwards(.5, 350, this);
            } else if (gamepad1.right_bumper) {
                _mecanum.TurnRight(.5, 350, this);
            } else if (gamepad1.left_bumper) {
                _mecanum.TurnLeft(.5, 350, this);
            } else if (gamepad1.y) {
                _mecanum.Move(.5, .5, .5, .5);
            } else if (gamepad1.a) {
                _mecanum.Move(-.5, -.5, -.5, -.5);
            } else if (gamepad1.x) {
                _mecanum.Move(-.5, .5, .5, -.5);
            } else if (gamepad1.b) {
                _mecanum.Move(.5, -.5, -.5, .5);
            } else {
                _mecanum.Stop();
            }

            telemetry.addData("Encoders lf, rf, lb, rb: ", "%d %d %d %d", robot.getFrontL().getCurrentPosition(),
                    robot.getFrontR().getCurrentPosition(), robot.getBackL().getCurrentPosition(),
                    robot.getBackR().getCurrentPosition());

            telemetry.addData("Power lf, rf, lb, rb: ", "%f %f %f %f", robot.getFrontL().getPower(),
                    robot.getFrontR().getPower(), robot.getBackL().getPower(),
                    robot.getBackR().getPower());

            telemetry.addData("Angle", "%f", GetImuAngle()); // 1300 - 1316

            telemetry.addData("Flip", "%d", robot.getFrontFlip().getCurrentPosition()); // 1300 - 1316
            telemetry.update();
        }
    }

    protected void InitGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
}
