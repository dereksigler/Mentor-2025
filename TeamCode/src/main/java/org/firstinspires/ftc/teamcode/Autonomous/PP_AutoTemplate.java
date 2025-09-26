package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/**
 * Pedro Pathing 2.0 – Linear Auto Template
 *
 * What you get:
 *  - Start pose + constraints wiring (via Constants)
 *  - Build one Path and two PathChains (preload, cycle, park)
 *  - followAndWait() helper for simple blocking segments
 *  - Optional FSM loop showing how to chain segments non-blocking
 *
 * Read more: Example Auto & API snippets on pedropathing.com.
 */
@Disabled
@Autonomous(name = "PP Auto Template (Linear)", group = "PP")
public class PP_AutoTemplate extends LinearOpMode {

    // --- Poses (field coordinates: 0..144, origin bottom-left; heading in radians) ---
    // Convert from your layout or Road Runner by adding +72 to x & y. :contentReference[oaicite:1]{index=1}
    private final Pose startPose   = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose   = new Pose(60,   85,  Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(37,  121,  Math.toRadians(0));
    private final Pose parkPose    = new Pose(14,   18,  Math.toRadians(270));

    private Follower follower;

    // Paths you’ll actually drive
    private Path      scorePreload;
    private PathChain grabPickup1;
    private PathChain parkChain;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1) Build follower from your Constants (PIDF, drivetrain, localizer, constraints)
        follower = Constants.createFollower(hardwareMap);  // from your project’s Constants
        // Docs example uses FollowerBuilder -> mecanum + localizer + constraints. :contentReference[oaicite:2]{index=2}

        // 2) Build paths BEFORE start for faster kick-off
        buildPaths();

        // 3) Set starting pose
        follower.setStartingPose(startPose);  // official method name in v2.0 examples. :contentReference[oaicite:3]{index=3}

        telemetry.addLine("Pedro 2.0 Auto Ready");
        telemetry.addData("Start", "(%.1f, %.1f) θ=%.1f°", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();

        // 4) Wait for start
        waitForStart();
        if (isStopRequested()) return;

        // === OPTION A: Simple, blocking segments ===
        followAndWait(scorePreload);
        followAndWait(grabPickup1);
        followAndWait(parkChain);

        // === OPTION B: FSM (non-blocking) ===
        // If you need to interleave mechanism actions or conditions, comment out Option A above
        // and use this instead.
        /*
        int state = 0;
        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case 0:
                    follower.followPath(scorePreload); // doesn’t block
                    state = 1;
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        // TODO: drop preload
                        follower.followPath(grabPickup1, true); // hold end point while acting
                        state = 2;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(parkChain);
                        state = 3;
                    }
                    break;
                case 3:
                    // done
                    break;
            }

            pushTelemetry();
            if (state == 3 && !follower.isBusy()) break;
        }
        */
    }

    // ----- Build your paths with official 2.0 API -----
    private void buildPaths() {
        // Straight line to score preload
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()); // v2.0 heading interp. :contentReference[oaicite:4]{index=4}

        // Single-segment PathChain back/forth to first pickup (line shown; curves also supported)
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build(); // v2.0 chain builder pattern. :contentReference[oaicite:5]{index=5}

        // Park: demonstrate a curve + line chain
        parkChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pose,
                        // control points for a gentle sweep; tune for your bot
                        new Pose(30, 100, pickup1Pose.getHeading()),
                        new Pose(20,  60, Math.toRadians(250)),
                        new Pose(18,  28, Math.toRadians(260))
                ))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), Math.toRadians(260))
                .addPath(new BezierLine(new Pose(18, 28, Math.toRadians(260)), parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(260), parkPose.getHeading())
                .build();
    }

    // ----- Helpers -----

    /** Follow a Path or PathChain and block until it’s finished, updating Pedro and telemetry. */
    private void followAndWait(Object pathOrChain) {
        if (pathOrChain == null) return;
        if (pathOrChain instanceof Path) {
            follower.followPath((Path) pathOrChain);
        } else if (pathOrChain instanceof PathChain) {
            follower.followPath((PathChain) pathOrChain);
        } else return;

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            pushTelemetry();
        }
    }

    private void pushTelemetry() {
        Pose p = follower.getPose();
        telemetry.addLine("== Pedro 2.0 ==");
        telemetry.addData("x", "%.1f", p.getX());
        telemetry.addData("y", "%.1f", p.getY());
        telemetry.addData("θ", "%.1f°", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }
}
