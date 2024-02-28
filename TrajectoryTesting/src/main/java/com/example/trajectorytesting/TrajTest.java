package com.example.trajectorytesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);


        TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(60, 6, 12);
        TrajectoryAccelerationConstraint accCon = SampleMecanumDrive.getAccelerationConstraint(40);
        TrajectoryVelocityConstraint velConPixel = SampleMecanumDrive.getVelocityConstraint(60, 6, 12);
        TrajectoryAccelerationConstraint accConPixel = SampleMecanumDrive.getAccelerationConstraint(40);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(18, -64, Math.toRadians(90)))
                                        .waitSeconds(10)
                                        .lineTo(new Vector2d(18, -30), velCon, accCon)
                                        .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)), velCon, accCon)
                                        .lineTo(new Vector2d(48, -64), velCon, accCon)
                                    .build()
                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(18, -64, Math.toRadians(90)))
                                .waitSeconds(10)
                                .splineToLinearHeading(new Pose2d(10, -31, Math.toRadians(180)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(48, -30, Math.toRadians(180)), velCon, accCon)
                                .lineTo(new Vector2d(48, -64), velCon, accCon)
                                .build()
                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(18, -64, Math.toRadians(90)))
                                .waitSeconds(10)
                                .lineTo(new Vector2d(25, -38), velCon, accCon)
                                .lineToSplineHeading(new Pose2d(48, -42, Math.toRadians(180)), velCon, accCon)
                                .lineTo(new Vector2d(48, -64), velCon, accCon)
                                .build()
                );

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(-36, -30), velCon, accCon) // drive to team prop
                                .lineTo(new Vector2d(-36, -36), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-42, -42), Math.toRadians(180), velCon, accCon)
                                .lineTo(new Vector2d(-52, -42), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-56, -36), Math.toRadians(90), velCon, accCon)
                                .lineTo(new Vector2d(-56, -16), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-52, -8), Math.toRadians(0), velCon, accCon)
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(180)), velCon, accCon)
                                .lineTo(new Vector2d(30, -8), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(52, -36), Math.toRadians(270), velCon, accCon)
                                .build()
                );

        RoadRunnerBotEntity myBot5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                .waitSeconds(10)
                                .lineToLinearHeading(new Pose2d(-42, -34, Math.toRadians(120)), velCon, accCon)
                                .lineTo(new Vector2d(-36, -34), velCon, accCon)
                                .lineToSplineHeading(new Pose2d(-32, -34, Math.toRadians(180)), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-29, -24), Math.toRadians(90), velCon, accCon)
                                .lineTo(new Vector2d(-29, -16), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-20, -8), Math.toRadians(0), velCon, accCon)
                                .lineTo(new Vector2d(30, -8), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(52, -30), Math.toRadians(270), velCon, accCon)
                                .build()
                );

        RoadRunnerBotEntity myBot6 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-42, -64, Math.toRadians(90)))
                                .waitSeconds(10)
                                .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(0)), Math.toRadians(0), velCon, accCon)
                                .lineTo(new Vector2d(-36, -34), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-38, -32), Math.toRadians(90), velCon, accCon)
                                .lineTo(new Vector2d(-38, -16), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(-34, -8), Math.toRadians(0), velCon, accCon)
                                .lineTo(new Vector2d(30, -8), velCon, accCon)
                                .splineToConstantHeading(new Vector2d(52, -42), Math.toRadians(270), velCon, accCon)
                                .build()
                );

        RoadRunnerBotEntity myBotStackTest = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(18, -64, Math.toRadians(90)))
                                .lineTo(new Vector2d(18, -30), velConPixel, accConPixel)
                                .lineToLinearHeading(new Pose2d(56.25, -36, Math.toRadians(180)), velConPixel, accConPixel)
                                .lineTo(new Vector2d(53, -37))
                                .lineToConstantHeading(new Vector2d(30, -10))
                                .lineToLinearHeading(new Pose2d(-53, 5, Math.toRadians(215)), velConPixel, accConPixel)
                                .waitSeconds(3)
                                .lineToSplineHeading(new Pose2d(-47, 5-20.0/83.0*6.0, Math.toRadians(180)), velConPixel, accConPixel)
                                .lineToConstantHeading(new Vector2d(30, -15), velConPixel, accConPixel)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(1f)
                .addEntity(myBot4)
                .start();
    }
}