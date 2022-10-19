package org.firstinspires.ftc.teamcodeAS;

import org.firstinspires.ftc.teamcodeAS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeAS.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="SplineAuto", group="Robot")
public class SplineAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(24)
                .strafeRight(48)
                .back(24)
                .strafeLeft(12)
                .waitSeconds(2)
                .strafeRight(12)
                .forward(48)
                .strafeLeft(12)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

