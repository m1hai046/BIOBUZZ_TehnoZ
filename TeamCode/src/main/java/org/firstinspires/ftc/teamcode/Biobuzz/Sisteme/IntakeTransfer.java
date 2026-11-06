package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class IntakeTransfer {

    public CachingDcMotorEx motorIntake, motorTransfer;
    public CachingServo rampa;

    public boolean flower = false;

    public static boolean reset = true;
    public static int confirmare = 0;
    public static double thresholdAmps = 5;

    /// POS RAMPA

    public static double rampaJos = 0.3;
    public static double rampaSus = 0.7;

    /// VARIABILE INTAKE
    public static double powIntake = 0.8;
    public static double powReverse = -1;
    public static double powOff = 0;


    /// VARIABILE TRANSFER
    public static double powTransferCollect = 0.4;
    /// gate  inchis
    public static double powTransferShoot = 0.95;   ///gate deschis


    public IntakeTransfer(HardwareMap hardwareMap){
        motorIntake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MINT"));
        motorTransfer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MTR"));
        rampa = new CachingServo(hardwareMap.get(CachingServo.class, "Rampa"));


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
        rampa.setPosition(rampaSus);
    }
    public void Reverse(){
        motorIntake.setPower(powReverse);
        motorTransfer.setPower(powReverse);
        rampa.setPosition(rampaSus);
    }
    public void Shoot(){
        motorIntake.setPower(powIntake);
        motorTransfer.setPower(powTransferShoot);
    }
    public void RampaSus(){
        rampa.setPosition(rampaSus);
        flower = true;
    }
    public void RampaJos(){
        rampa.setPosition(rampaJos);
        flower = false;
    }


    public double getAmps() {
        return motorIntake.getCurrent(CurrentUnit.AMPS);
    }

    public boolean hasArtifact() {
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
