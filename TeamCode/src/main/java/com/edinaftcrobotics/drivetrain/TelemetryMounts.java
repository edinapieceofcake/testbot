package com.edinaftcrobotics.drivetrain;

public class TelemetryMounts {

//    Diameter of the omniwheels
    public static final double OMNI_DIAMETER = 2.875;
//    Diameter of the mecanum wheels
    public static final double MECANUM_DIAMETER = 4;
//    How many ticks it is for a full 360
    public static final double CPR = 200;
//    The distance between the two omniwheels
    public static final double DIAMETER = Double.NaN; // this is currently unknown, so for now its just NaN

    private double x;
    private double y;
    private double r;

    public TelemetryMounts(){
        set(0, 0,0);
    }

//    If the user wants to start at a specific location
    public TelemetryMounts(double x, double y, double r){
        set(x, y, r);
    }

    public void set(double x, double y, double r){
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public void update(int rightEncoderTranslate, int leftEncoderTranslate, int strafeEncoderTranslate){

//        This is to keep track of the rotation of the robot
//        if the imu is more reliable than this, then use the other method
        int difference = leftEncoderTranslate - rightEncoderTranslate;
        double distance = toDistance(difference);
        r += (distance / (DIAMETER * Math.PI)) * 360;

//        This double modulo is to loop the negatives as well
        r = ((r % 360) + 360) % 360;

//        This are our local velocities
        double forward, strafe;

        strafe = toDistance(strafeEncoderTranslate);
        forward = toDistance((rightEncoderTranslate + leftEncoderTranslate) / 2);

//        Now we are translating our local velocities to polar coordinates
        double radius, theta;

        radius = Math.sqrt(Math.pow(strafe, 2) + Math.pow(forward, 2));
        theta = Math.atan2(forward, strafe);

        x += Math.cos(Math.toRadians(r) + theta) * radius;
        y += Math.sin(Math.toRadians(r) + theta) * radius;


    }


//    This is for an imu input rather than keeping track of rotation as well
    public void update(int rightEncoderTranslate, int leftEncoderTranslate, int strafeEncoderTranslate, double heading){

        r = heading;

//        This double modulo is to loop the negatives as well
        r = ((r % 360) + 360) % 360;

//        This are our local velocities
        double forward, strafe;

        strafe = toDistance(strafeEncoderTranslate);
        forward = toDistance((rightEncoderTranslate + leftEncoderTranslate) / 2);

//        Now we are translating our local velocities to polar coordinates
        double radius, theta;

        radius = Math.sqrt(Math.pow(strafe, 2) + Math.pow(forward, 2));
        theta = Math.atan2(forward, strafe);


//        This will be how many divisions of the arc will occur
//        The more iterations, the better the arc, but the more processing power used
//        This only really be high if the encoders are called infrequently
        int resolution = 1;

        for (int i = 0; i < resolution; i++) {
            x += Math.cos(Math.toRadians(r) + (theta / resolution) * (i + 1)) * (radius / resolution);
            y += Math.sin(Math.toRadians(r) + (theta / resolution) * (i + 1)) * (radius / resolution);
        }



    }

    private double toDistance(int ticks){
        return (ticks / CPR) * OMNI_DIAMETER * Math.PI;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getHeading(){
        return r;
    }

}
