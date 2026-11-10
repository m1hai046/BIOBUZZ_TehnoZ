package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class IntakeTransfer {

    public static CachingDcMotorEx motorIntake, motorTransfer;
    public CachingServo rampaLeft, rampaRight;

    public static double rampaJos = 0.04;
    public static double rampaSus = 0.73;
    public boolean flower = false;



    /// VARIABILE INTAKE
    public static double powIntake = 0.8;
    public static double powReverse = -1;
    public static double powOff = 0;

    public static boolean reset = true;
    public static int confirmare = 0;
    public static double thresholdAmps = 6;


    /// VARIABILE TRANSFER
    public static double powTransferCollect = 0.7;
    /// gate  inchis
    public static double powTransferShoot = 0.85;   ///gate deschis


    public IntakeTransfer(HardwareMap hardwareMap){
        motorIntake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MINT"));
        motorTransfer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MTR"));
        rampaLeft = new CachingServo(hardwareMap.get(Servo.class, "RampaRight"));
        rampaRight = new CachingServo(hardwareMap.get(Servo.class, "RampaLeft"));

        motorIntake.setDirection(CachingDcMotorEx.Direction.REVERSE);
        motorTransfer.setDirection(CachingDcMotorEx.Direction.FORWARD);
    }

    public void Collect(){
        motorIntake.setPower(powIntake);
        motorTransfer.setPower(powTransferCollect);
    }
    public void Off(){
        motorIntake.setPower(powOff);
        motorTransfer.setPower(powOff);
    }
    public void Reverse(){
        motorIntake.setPower(powReverse);
        motorTransfer.setPower(powReverse);
    }
    public void Shoot(){
        motorIntake.setPower(powIntake);
        motorTransfer.setPower(powTransferShoot);
    }

    public void RampaSus(){
        rampaRight.setPosition(rampaSus);
        rampaLeft.setPosition(rampaSus);
        flower = false;
    }
    public void RampaJos(){
        rampaRight.setPosition(rampaJos);
        rampaLeft.setPosition(rampaJos);
        flower = true;
    }


    public static double getAmps() {
        return motorIntake.getCurrent(CurrentUnit.AMPS);
    }

    /// de terminat
    public static boolean hasArtifact() {
        if (!reset) {
            confirmare = 0;
            return false;
        } else if (reset) {
            if (getAmps() > thresholdAmps) {
                confirmare++;
            } else {
                confirmare = 0;
            }
        }
        return confirmare > 4;
    }
}
