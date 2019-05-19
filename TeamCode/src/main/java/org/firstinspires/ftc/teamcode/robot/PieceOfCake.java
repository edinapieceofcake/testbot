package org.firstinspires.ftc.teamcode.robot;
//imports
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class PieceOfCake {
    private DcMotor FrontR = null;
    private DcMotor FrontL = null;
    private DcMotor BackR = null;
    private DcMotor BackL = null;
    private DcMotor BackLift = null;
    private DcMotor FrontLift = null;
    private DcMotor Slide = null;
    private DcMotor FrontFlip = null;
    private CRServo Intake = null;
    private Servo TopFlip = null;
    private HardwareMap hwMap = null;

    //constructor
    public PieceOfCake() {

    }
    public void init(HardwareMap ahwMap) {
        DcMotor dcMotor = null;
        Servo servo = null;
        CRServo crServo = null;

        hwMap = ahwMap;

        dcMotor = hwMap.dcMotor.get("fl");
        SetFrontL(dcMotor);

        dcMotor = hwMap.dcMotor.get("fr");
        SetFrontR(dcMotor);

        dcMotor = hwMap.dcMotor.get("bl");
        SetBackL(dcMotor);

        dcMotor = hwMap.dcMotor.get("br");
        SetBackR(dcMotor);
    }


    public DcMotor getFrontL() {
        return FrontL;
    }

    public DcMotor getFrontR() {
        return FrontR;
    }

    public DcMotor getBackL() {
        return BackL;
    }

    public DcMotor getBackR() {
        return BackR;
    }

    public DcMotor getBackLift() { return BackLift; }

    public DcMotor getFrontLift() { return FrontLift; }

    public DcMotor getSlide() { return Slide; }

    public DcMotor getFrontFlip() { return FrontFlip; }

    public CRServo getIntake() { return Intake; }

    public Servo getTopFlip() { return TopFlip; }

    //public CRServo getLockServo() { return LockServo; }


    private void SetFrontL(DcMotor dcMotor) {FrontL = dcMotor; }
    private void SetFrontR(DcMotor dcMotor) {FrontR = dcMotor; }
    private void SetBackL(DcMotor dcMotor) {BackL = dcMotor; }
    private void SetBackR(DcMotor dcMotor) {BackR = dcMotor; }
    private void SetBackLift(DcMotor dcMotor) {BackLift = dcMotor; }
    private void SetFrontLift(DcMotor dcMotor) {FrontLift = dcMotor; }
    private void SetSlide(DcMotor dcMotor) {Slide = dcMotor; }
    private void SetFrontFlip(DcMotor dcMotor) {FrontFlip = dcMotor; }
    private void SetIntake(CRServo crServo) {Intake = crServo; }
    private void SetTopFlip(Servo servo) {TopFlip = servo; }
    //private void SetLockServo(CRServo servo) { LockServo = servo; }

    public void setMotorPower(double fl, double fr, double bl, double br){
        getFrontL().setPower(fl);
        getFrontR().setPower(fr);
        getBackL().setPower(bl);
        getBackR().setPower(br);
    }
}


