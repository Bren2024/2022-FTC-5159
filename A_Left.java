package org.firstinspires.ftc.teamcodeAS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodeAS.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcodeAS.trajectorysequence.TrajectorySequence;

@Autonomous(name="LeftAuto", group="Robot")
public class A_Left extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Monkey monkey = new Monkey();
//        monkey.init(this);

        Pose2d startPose = new Pose2d(4, -3, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(0, () -> {
//                    monkey.autoServoOp(this, true);
//                })
                .lineToConstantHeading(new Vector2d(0, 0))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(24, 0))
//                .addDisplacementMarker(() -> {
//                    monkey.autoExtendMonkeyUp(this, 24);
//                })
                .lineToConstantHeading(new Vector2d(24, 48))
                .lineToConstantHeading(new Vector2d(36, 48))
//                .addDisplacementMarker(() -> {
//                    monkey.(this, false);
//                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(0, 48))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}

