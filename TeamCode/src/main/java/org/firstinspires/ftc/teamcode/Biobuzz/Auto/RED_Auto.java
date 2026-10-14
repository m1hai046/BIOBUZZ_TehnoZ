package org.firstinspires.ftc.teamcode.Biobuzz.Auto;


import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Configurable
@Autonomous(name="RED_Auto", group="Auto")

public class RED_Auto extends LinearOpMode {

    public Pose startPose = new Pose(32,8.5, Math.toRadians(180));
    public Pose spikePose = new Pose(9.1,8.5, Math.toRadians(181));
    public Follower follower;
    public ElapsedTime pathTimer;
    public IntakeTransfer intakeTransfer;
    public Shooter shooter;
    public Turret turret;
    int pathState = 0;

    public void runOpMode() throws InterruptedException {

        intakeTransfer = new IntakeTransfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        waitForStart();
        pathTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {


            Pose currentPose = follower.pose();

            shooter.update_shooter(
                    currentPose.x(),
                    currentPose.y()
            );

            turret.update_turret(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading()
            );
            follower.update();

            autonomousPathUpdate();
        }

    }



    private Path collectSpike() {
        return line(startPose, spikePose).linear(startPose, spikePose);
    }

    public void setPathState(int n){
        pathTimer.reset();
        pathState = n;

    }
    public void autonomousPathUpdate(){
        switch(pathState){

            case 0:
                if (!follower.following() && pathTimer.seconds() > 1.0){
                    intakeTransfer.Collect();
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(1);
                }
                break;

                case 1:
                if (!follower.following() && shooter.state == Shooter.State.RUNNING){
                    follower.follow(collectSpike());
                    setPathState(2);
                }
                break;

                case 2:
                if (!follower.following() && shooter.state == Shooter.State.SHOOTING){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(-1);
                }
                break;

        }
    }
}
