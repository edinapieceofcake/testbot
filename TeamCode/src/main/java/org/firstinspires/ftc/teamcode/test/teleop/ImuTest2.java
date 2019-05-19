package org.firstinspires.ftc.teamcode.test.teleop;

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

@TeleOp(name="Test: IMU2", group ="Autonomous Test")
@Disabled
public class ImuTest2 extends LinearOpMode {

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

        while (opModeIsActive()) {

            final double heading = this.getRelativeHeading();

            boolean doneTurn1 = this.gyroCorrect(-90.0, 1., heading, 0.1, 0.3) > 10;
            this.telemetry.addData("Heading", heading);
            this.telemetry.addData("Done Turn", doneTurn1);
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

    private int correctCount = 0;
    /*
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        telemetry.addData("Delta", delta);
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            this.correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0) gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            this.correctCount++;
            this.turn(0.0);
        }
        return this.correctCount;
    }

    public void turn(double sPower) {
        telemetry.addData("Wheel Power", sPower);
        //front_left.setPower(- sPower);
        //back_left.setPower(- sPower);
        //front_right.setPower(+ sPower);
        //back_right.setPower(+ sPower);
    }
}