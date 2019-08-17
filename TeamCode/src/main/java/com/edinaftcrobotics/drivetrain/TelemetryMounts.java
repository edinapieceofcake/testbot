package com.edinaftcrobotics.drivetrain;

public class TelemetryMounts {

//    Diameter of the omniwheels
    private double omniDiameter;
//    Diameter of the mecanum wheels
    private double mecanumDiameter;
//    How many ticks it is for a full 360
    private double CPR;
//    The distance between the two omniwheels
    private double diameter;

    private double x;
    private double y;
    private double r;

//    If the user wants to start at a specific location
    public TelemetryMounts(double x, double y, double r, double omniDiameter, double mecanumDiameter, double CPR, double diameter){
        set(x, y, r);
        this.omniDiameter = omniDiameter;
        this.mecanumDiameter = mecanumDiameter;
        this.CPR = CPR;
        this.diameter = diameter;
    }

    public TelemetryMounts(double omniDiameter, double mecanumDiameter, double CPR, double diameter){
        set(x, y, r);
        this.omniDiameter = omniDiameter;
        this.mecanumDiameter = mecanumDiameter;
        this.CPR = CPR;
        this.diameter = diameter;
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
        r += (distance / (diameter * Math.PI)) * 360 * 0.5;

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

//        These are our local velocities
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
        return (ticks / CPR) * omniDiameter * Math.PI;
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
