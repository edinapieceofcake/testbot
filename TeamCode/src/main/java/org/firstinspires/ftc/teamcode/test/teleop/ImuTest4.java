package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.controller.MiniPID;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Test: IMU4", group ="Autonomous Test")
@Disabled
public class ImuTest4 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare DC Motors.
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor back_right;

    //Declare Servos

    //Declare Sensors
    BNO055IMU imu;

    //User Generated Values
    double headingResetValue;

    @Override
    public void runOpMode() {

        PieceOfCake robot = new PieceOfCake();
        MiniPID pid = new MiniPID(1.5, .05, .3);

        //robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //DC MOTORS
        //front_left = robot.getFrontL();
        //back_left = robot.getBackL();
        //front_right = robot.getFrontR();
        //back_right = robot.getBackR();

        //SET THE DRIVE MOTOR DIRECTIONS
        //front_left.setDirection(DcMotor.Direction.FORWARD);
        //back_left.setDirection(DcMotor.Direction.FORWARD);
        //front_right.setDirection(DcMotor.Direction.REVERSE);
        //back_right.setDirection(DcMotor.Direction.REVERSE);

        //SET THE RUN MODE FOR THE MOTORS
        //back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //SERVOS

        //SENSORS
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);

        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //INITIALIZATIONS
        this.headingResetValue = this.getAbsoluteHeading();

        waitForStart();
        runtime.reset();

        pid.setSetpoint(90);
        while (opModeIsActive()) {

            double heading = this.getRelativeHeading();

            this.telemetry.addData("Heading", heading);
            this.telemetry.addData("Power", pid.getOutput(heading));
            this.telemetry.update();
        }
    }


    //FUNCTIONS

    //GYRO TURNING
    private double getAbsoluteHeading(){
        return this.imu.getAngularOrientation(AxesReference.EXTRINSIC.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading(){
        return this.getAbsoluteHeading() - this.headingResetValue;
    }


    public void turn(double sPower) {
        telemetry.addData("Wheel Power", sPower);
        //front_left.setPower(- sPower);
        //back_left.setPower(- sPower);
        //front_right.setPower(+ sPower);
        //back_right.setPower(+ sPower);
    }
}