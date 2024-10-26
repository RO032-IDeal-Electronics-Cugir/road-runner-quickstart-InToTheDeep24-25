
package org.firstinspires.ftc.teamcode.IDL;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose  = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive  = new MecanumDrive(hardwareMap, initialPose);

        Parts.HSlide hSlide           = new Parts.HSlide(hardwareMap);
        Parts.IntakeArm intakeArm     = new Parts.IntakeArm(hardwareMap);
        Parts.IntakeMotor intakeMotor = new Parts.IntakeMotor(hardwareMap);
        Parts.Lift lift               = new Parts.Lift(hardwareMap);

        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(34,-10),Math.toRadians(180))
                .build();

        Action trajectoryAction2;
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(1)
                .build();

        Action trajectoryActionWait0_5;
        trajectoryActionWait0_5 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", drive.pose);
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        lift.liftUpdatePIDF(),
                        new SequentialAction(
                                new ParallelAction(
                                        lift.liftUp(),
                                        trajectoryAction1
                                ),
                                new SleepAction(1),
                                lift.liftDeploy(),
                                new ParallelAction(
                                        lift.liftDown(),
                                        lift.liftLoad(),
                                        trajectoryAction2
                                )
                        )

                )
        );

    }
}
///////////////////////////
