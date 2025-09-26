package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A thin wrapper so your OpModes don’t depend on a specific Pedro Pathing fork’s API.
 * Map each TODO to your actual classes/methods.
 */
public class PedroDriveAdapter {

    // === Replace these with your library’s types ===
    // e.g., com.pedro.pathing.Follower, com.pedro.pathing.PathBuilder, com.pedro.math.Pose2d, etc.
    private final Telemetry telemetry;
    private final HardwareMap hw;

    // Example: your follower/drive + pose/localizer
    private final Object follower; // TODO: replace with your follower type

    public PedroDriveAdapter(HardwareMap hw, Telemetry telemetry) {
        this.hw = hw;
        this.telemetry = telemetry;

        // TODO: construct your drive/follower/localizer here.
        // follower = new Follower(hw, /* params */);
        follower = new Object();
    }

    /** Set the starting field pose in inches + degrees. */
    public void setPoseInches(double xIn, double yIn, double headingDeg) {
        // TODO: convert to your lib’s pose and set it
        // follower.setPose(new Pose2d(inToM(xIn), inToM(yIn), Math.toRadians(headingDeg)));
    }

    /** Begin following a path. */
    public void followPath(PedroPath path) {
        // TODO: cast and start the follower with your path type
        // follower.followPath(path.impl);
    }

    /** Update follower (call in loop). */
    public void update() {
        // TODO: follower.update();
    }

    /** Tell whether the follower finished the active path. */
    public boolean isFinished() {
        // TODO: return follower.isFinished();
        return true;
    }

    /** Stop drive outputs but keep pose/localizer alive. */
    public void stopMotion() {
        // TODO: follower.stop();
    }

    /** Stop everything at end of OpMode. */
    public void stop() {
        stopMotion();
        // TODO: extra shutdown if needed
    }

    /** Basic status string for telemetry. */
    public String getState() {
        // TODO: return follower.getState().toString();
        return "IDLE";
    }

    /** Current pose for telemetry display (returns inches/degrees). */
    public PedroPose getPose() {
        // TODO: read from your localizer/follower
        // Pose2d p = follower.getPose();
        // return new PedroPose(mToIn(p.position.x), mToIn(p.position.y), p.heading);
        return new PedroPose(0,0,0);
    }

    /** Position error in inches for telemetry. */
    public double getPosErrorInches() {
        // TODO: return mToIn(follower.getPosError());
        return 0.0;
    }

    /** Heading error in degrees for telemetry. */
    public double getHeadingErrorDeg() {
        // TODO: return Math.toDegrees(follower.getHeadingError());
        return 0.0;
    }

    /** Start a new path builder already bound to this drive/localizer. */
    public PedroPathBuilder newPathBuilder() {
        // Return a builder that internally builds your concrete path type.
        return new PedroPathBuilder();
    }

    // Unit helpers if your fork uses meters internally
    private static double inToM(double in) { return in * 0.0254; }
    private static double mToIn(double m) { return m / 0.0254; }
}

/** Simple pose container for telemetry (heading in radians). */
class PedroPose {
    public final double xInches, yInches, headingRad;
    public PedroPose(double xInches, double yInches, double headingRad) {
        this.xInches = xInches; this.yInches = yInches; this.headingRad = headingRad;
    }
}

/**
 * Library-agnostic path and builder wrappers.
 * Replace internals to call your actual PathBuilder API (splineTo, lineTo, markers, waits, etc.).
 */
class PedroPath {
    public Object impl; // TODO: store your concrete path object
    PedroPath(Object impl) { this.impl = impl; }
}

class PedroPathBuilder {
    // Store your real builder inside
    private final Object realBuilder; // TODO: PathBuilder

    PedroPathBuilder() {
        // TODO: realBuilder = new PathBuilder();
        realBuilder = new Object();
    }

    public PedroPathBuilder setTangentDeg(double deg) {
        // TODO: realBuilder.setTangent(Math.toRadians(deg));
        return this;
    }

    public PedroPathBuilder splineToInches(double xIn, double yIn, double tangentDegAtEnd) {
        // TODO: realBuilder.splineTo(inToM(xIn), inToM(yIn), Math.toRadians(tangentDegAtEnd));
        return this;
    }

    public PedroPathBuilder lineToInches(double xIn, double yIn) {
        // TODO: realBuilder.lineTo(inToM(xIn), inToM(yIn));
        return this;
    }

    public PedroPathBuilder waitSeconds(double sec) {
        // TODO: realBuilder.wait(sec);
        return this;
    }

    public PedroPathBuilder withMarker(String name, Runnable action) {
        // TODO: attach a temporal/spatial marker with callback
        // realBuilder.addMarker(name, action);
        return this;
    }

    public PedroPath build() {
        // TODO: Object path = realBuilder.build();
        Object path = new Object();
        return new PedroPath(path);
    }

    private static double inToM(double in) { return in * 0.0254; }
}
