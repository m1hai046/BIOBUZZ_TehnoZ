package org.firstinspires.ftc.teamcode.Biobuzz.Auto.Alianta;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter.Shooter_Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Paths_Auto_Alianta {
    public static Pose startPose,
            collectgarden,
            shootgarden,
            shootgardenControl1,
            collectflowerStart,
            collectflower,
            shootflower,
            shootflowerControl1,
            collectflower2p1,
            collectflower2p2Start,
            collectflower2p2,
            shootflower2Start,
            shootflower2,
            park,
            parkControl1;

    public static Follower paths_follower;
    public static ElapsedTime paths_pathTimer;
    public static IntakeTransfer paths_intakeTransfer;
    public static Shooter paths_shooter;
    public static Turret paths_turret;
    static int pathState = 0;

    public static void initPathsAutoSolo(Follower follower, ElapsedTime pathTimer, IntakeTransfer  intakeTransfer, Shooter  shooter, Turret turret){
        paths_follower = follower;
        paths_pathTimer = pathTimer;
        paths_intakeTransfer =  intakeTransfer;
        paths_shooter =  shooter;
        paths_turret = turret;
        pathState = 0;
    }

    public static void buildPoses() {
        PoseFactory poseFactory = PoseFactory.degrees();
        if (Globals.alliance == Globals.Alliance.BLUE) {
            poseFactory = poseFactory.rotateAround(70.5, 70.5, 180);
        }

        startPose = poseFactory.of(38, 8.5, 180);
        collectgarden = poseFactory.of(6.5, 8, 181);
        shootgarden = poseFactory.of(48, 120, 55.8403);
        shootgardenControl1 = poseFactory.of(29, 90, 0);
        collectflowerStart = poseFactory.of(48, 118, 90);
        collectflower = poseFactory.of(48, 130, 90);
        shootflower = poseFactory.of(60, 16.2488, 86.2317);
        shootflowerControl1 = poseFactory.of(65.6875, 102.6006, 0);
        collectflower2p1 = poseFactory.of(23, 46, 141.1977);
        collectflower2p2Start = poseFactory.of(23, 46, 180);
        collectflower2p2 = poseFactory.of(11.5, 47, 180);
        shootflower2Start = poseFactory.of(13.5, 47, 180);
        shootflower2 = poseFactory.of(49.7554, 17.3145, 120);
        park = poseFactory.of(10.4861, 94.1629, 99.1834);
        parkControl1 = poseFactory.of(17.1881, 52.7074, 0);
    }

    public static Path collectgarden() {
        return Paths.line(startPose, collectgarden).linear(startPose, collectgarden);
    }

    public static Path shootgarden() {
        return Paths.curve(collectgarden, shootgardenControl1, shootgarden).tangent();
    }

    public static Path collectflower() {
        return Paths.line(collectflowerStart, collectflower).linear(collectflowerStart, collectflower);
    }

    public static Path shootflower() {
        return Paths.curve(collectflower, shootflowerControl1, shootflower).reverseTangent();
    }

    public static Path collectflower2p1() {
        return Paths.line(shootflower, collectflower2p1).tangent();
    }

    public static Path collectflower2p2() {
        return Paths.line(collectflower2p2Start, collectflower2p2).linear(collectflower2p2Start, collectflower2p2);
    }

    public static Path shootflower2() {
        return Paths.line(shootflower2Start, shootflower2).linear(shootflower2Start, shootflower2);
    }

    public static Path park() {
        return Paths.curve(shootflower2, parkControl1, park).tangent();
    }

    public static void setPathState(int n){
         paths_pathTimer.reset();
        pathState = n;

    }
    public static void autonomousPathUpdate(){
        switch(pathState){

            case 0:
                if (!paths_follower.following()){
                    TurretConstants.offset = 5;
                    Shooter.rpmOffset = - 20;
                     paths_shooter.state =  Shooter.State.RUNNING;
                     paths_intakeTransfer.RampaSus();
                     paths_intakeTransfer.Off();
                    setPathState(11);
                }
                break;

            case 11:///da shoot la preload
                if (! paths_follower.following() &&  paths_pathTimer.milliseconds() > 2000){
                     paths_intakeTransfer.Shoot();
                     paths_shooter.state =  Shooter.State.SHOOTING;
                    setPathState(1);
                }
                break;

            case 1:
                if ( paths_shooter.state !=  Shooter.State.SHOOTING){
                     paths_intakeTransfer.Collect();
                     paths_follower.follow(collectgarden());
                     Shooter.rpmOffset = - 30;
                    setPathState(2);
                }
                break;

            case 2:
                if ( paths_pathTimer.milliseconds() > 3200){
                     paths_follower.follow(shootgarden());
                    TurretConstants.offset = -5;
                    setPathState(12);
                }
                break;

            case 12:
                if (! paths_follower.following() &&  paths_pathTimer.milliseconds() > 3500){
                     paths_intakeTransfer.Shoot();
                     paths_shooter.state =  Shooter.State.SHOOTING;
                    paths_intakeTransfer.RampaJos();
                    setPathState(3);
                }
                break;

            case 3:
                if ( paths_shooter.state !=  Shooter.State.SHOOTING){
                     paths_intakeTransfer.Collect();
                     paths_follower.follow(collectflower());
                    setPathState(4);
                }
                break;

            case 4:
                if ((! paths_follower.following() &&  paths_pathTimer.milliseconds() > 2600) ||   paths_pathTimer.milliseconds() > 2800){
                     paths_follower.follow(shootflower());
                    setPathState(13);
                }
                break;

            case 13:
                if (! paths_follower.following() &&  paths_pathTimer.milliseconds() > 3000){
                     paths_intakeTransfer.RampaSus();
                     paths_intakeTransfer.Shoot();
                     paths_shooter.state =  Shooter.State.SHOOTING;
                    setPathState(5);
                }
                break;

            case 5:
                if ( paths_shooter.state !=  Shooter.State.SHOOTING){
                     paths_follower.follow(collectflower2p1());
                     paths_intakeTransfer.Reverse();
                    paths_intakeTransfer.RampaJos();
                    setPathState(6);
                }
                break;

            case 6:
                if (! paths_follower.following()){
                     paths_intakeTransfer.Collect();
                     paths_follower.follow(collectflower2p2());
                    TurretConstants.offset = -15;
                    Shooter.rpmOffset = 0;
                    setPathState(7);
                }
                break;

            case 7:
                if ((! paths_follower.following() &&  paths_pathTimer.milliseconds() > 3500) ||  paths_pathTimer.milliseconds() > 3700){
                    paths_follower.follow(shootflower2());
                    setPathState(8);
                }
                break;

            case 8:
                if (! paths_follower.following() &&  paths_pathTimer.milliseconds() > 3000){
                     paths_intakeTransfer.RampaSus();
                     paths_intakeTransfer.Shoot();
                     paths_shooter.state =  Shooter.State.SHOOTING;
                    setPathState(9);
                }
                break;

            case 9:
                if ( paths_shooter.state !=  Shooter.State.SHOOTING){
                     paths_follower.follow(park());
                     paths_intakeTransfer.RampaSus();
                     paths_intakeTransfer.Off();
                    setPathState(-1);
                }
                break;
        }
    }
}