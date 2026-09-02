package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.*;

@Disabled
@Autonomous(group="Pedro")
public class IvyAuto extends OpMode {

    private Follower follower;

    private PathChain forwards;

    private PathChain backwards;

    DcMotorEx    armMotor;
    Servo handServo;

    Command auto;
    Command goForthAndGrab;
    Command returnAndRelease;


    @Override
    public void init() {

        /*
         * Initialize follower, set starting pose, and create PathChains
         */

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(48, 48, Math.toRadians(90)));

        forwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 48.000),
                        new Pose(47.241, 79.399),
                        new Pose(96.759, 65.170),
                        new Pose(96.000, 96.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

        backwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(96.000, 96.000),
                        new Pose(95.336, 66.877),
                        new Pose(47.241, 78.545),
                        new Pose(48.000, 48.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

        /*
         * Reset the Ivy Scheduler. This is necessary because the methods of Scheduler are STATIC
         */
        Scheduler.reset();

        /*
         * Initialize the armMotor and handServo
         */
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        handServo = hardwareMap.get(Servo.class, "hand_servo");

        /*
         * Set up Ivy Commands.
         */
        goForthAndGrab = sequential(
                parallel(
                        follow(follower, forwards),
                        sequential(
                                waitUntil(()->follower.getPose().getX() > 72),
                                setArmPosition(2000)
                        )
                ),
                instant(()->handServo.setPosition(1)),
                waitMs(500),
                setArmPosition(0)
        );

        returnAndRelease = sequential(
                parallel(
                        follow(follower, backwards),
                        sequential(
                                waitUntil(()->follower.getPose().getX() < 72),
                                setArmPosition(2000)
                        )
                ),
                instant(()->handServo.setPosition(0)),
                waitMs(500),
                setArmPosition(0)
        );

        auto = Groups.loop(
                sequential(
                        goForthAndGrab,
                        turnTo(follower, Math.toRadians(270)),
                        returnAndRelease,
                        turnTo(follower, Math.toRadians(90))
                )
        );

    }

    public void start(){
        schedule(auto);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        Scheduler.execute();
        follower.update();
    }

    /*
     * Return a new command that runs the arm to the requested position.
     * Because the RUN_TO_POSITION mode is used for armMotor, it is not necessary
     * to set an Execute method for the Command.
     */
    private Command setArmPosition(int armPosition){
        return Command.build()
                .setStart(() ->
                {
                    armMotor.setTargetPosition(armPosition);
                    armMotor.setPower(0.5);
                })
                .setDone(()->!armMotor.isBusy());
    }

}