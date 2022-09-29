package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

@Autonomous(name="SplineAuto", group="Robot")
public class SplineAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(48, 24, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(24,0))
                .splineTo(new Vector2d(0, 24), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,0, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(24, 24, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(180))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

