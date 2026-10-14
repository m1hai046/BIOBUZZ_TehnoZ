package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.Gamepad;


@Configurable
public class Drive {



    public void pedro_RC(Follower follower, Gamepad gamepad1) {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);
    }

    public void pedro_FC(Follower follower, Gamepad gamepad1) {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading());

        follower.manual(powers);
    }
}
