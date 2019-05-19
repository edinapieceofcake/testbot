package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Mat;

@TeleOp(name = "Test: IMU3", group = "Test")
@Disabled
public class ImuTest3 extends LinearOpMode {
    private double KP;

    /**
     * The gain for the integral part of the controller.
     */
    private double KI;

    /**
     * The gain for the derivative part of the controller.
     */
    private double KD;

    /** The output from the proportional part of the controller. */
    private double error = 0;

    /**
     * The position the robot is trying to get to.
     */
    private double target = 0;

    /**
     * The ouput from the integral part of the controller.
     */
    private double integral = 0;

    /**
     * The output from the derivative part of the controller.
     */
    private double derivative = 0;

    /**
     * The time, in nanoseconds, of the last time of controller ran through the loop. Used when caluclating integral and derivative.
     */
    private long timeAtUpdate;

    private boolean integralSet = false;
    private boolean derivativeSet = false;
    private double derivativeAveraging = 0.95;
    private boolean processManualDerivative = false;

    private double maxErrorForIntegral = Double.POSITIVE_INFINITY;
    private double maxDerivative = Double.POSITIVE_INFINITY;

    private double lastHeading, thisHeading;
    private int rotations;

    private double nanoToUnit(long nano) {  //Used to convert nanoseconds to seconds
        return nano/1E9;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    /**
     * Gets the target value the system is currently trying to reach or maintain.
     * @return The current target value
     */
    public double getTarget() {
        return target;
    }

    public double output() {
        return KP*error+KI*integral+KD*derivative;
    }

    /**
     * Does all of the fancy math, requiring only an input into the proportional part of the controller.
     * @param input The proportional input into the fancy math, such as the heading from a gyro
     */
    public void input(double input) {
        long newTime = System.nanoTime();
        error = target-input;
        if (!integralSet) integral += Range.clip(error, -maxErrorForIntegral, maxErrorForIntegral)*nanoToUnit(newTime-timeAtUpdate);
        if (!derivativeSet) derivative = derivative*derivativeAveraging+(error/nanoToUnit(newTime-timeAtUpdate))*(1-derivativeAveraging);
        timeAtUpdate = newTime;
        integralSet = false;
        derivativeSet = false;
    }

    public void update(double unconvertedHeading) {
        thisHeading = unconvertedHeading;
        lastHeading = unconvertedHeading;
    }

    public double getConvertedHeading() {
        return thisHeading;
    }

    public void runOpMode() {
        BNO055IMUImpl imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Add calibration file?
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());

        maxErrorForIntegral = Math.abs(.002);
        KP = 1.5;
        KI = 0.05;
        KD = 0;

        waitForStart();

        target = -90;
        input(0);
        double output = output();
        telemetry.addData("Wheel turning power", "%f", output);
        telemetry.update();
        while (Math.abs(output) > 3) {
            update(imu.getAngularOrientation(AxesReference.EXTRINSIC.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Heading", "%f", getConvertedHeading());
            input(getConvertedHeading());
            output = output();
            telemetry.addData("Wheel turning power", "%f", output);
            telemetry.update();
        }
    }
}