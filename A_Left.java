package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="LeftAuto", group="Robot")
public class A_Left extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Monkey monkey = new Monkey();
        monkey.init(this);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(1, () -> {
                    monkey.autoExtendMonkeyUp(this, 12);
                })
                .lineToConstantHeading(new Vector2d(3, 0))
                .lineToConstantHeading(new Vector2d(3, -24))
                .lineToConstantHeading(new Vector2d(27, -24))
                .lineToConstantHeading(new Vector2d(27, -36))
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

