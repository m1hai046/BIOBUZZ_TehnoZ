package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
public class IntakeTransfer {

    public CachingDcMotorEx motorIntake, motorTransfer;

    /// VARIABILE INTAKE
    public static double powIntake = 0.9;
    public static double powReverse = -1;
    public static double powOff = 0;


    /// VARIABILE TRANSFER
    public static double powTransferCollect = 0.7; //0.35
    /// gate  inchis
    public static double powTransferShoot = 0.85;///0.6   ///gate deschis


    public IntakeTransfer(HardwareMap hardwareMap){
        motorIntake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MINT"));
        motorTransfer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MTR"));

        motorIntake.setDirection(CachingDcMotorEx.Direction.FORWARD);
        motorTransfer.setDirection(CachingDcMotorEx.Direction.REVERSE);
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
}
