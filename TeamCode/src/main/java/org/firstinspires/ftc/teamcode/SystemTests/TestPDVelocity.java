package org.firstinspires.ftc.teamcode.SystemTests;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Flywheel.FtclibVelocityPD;

/**
 * PD Velocity Tester for a single motor using FtclibVelocityPD (FTCLib-only controller).
 *
 * Controls (gamepad1):
 *  - RIGHT_BUMPER:  +100 tps
 *  - LEFT_BUMPER:   -100 tps
 *  - DPAD_UP:       +25 tps
 *  - DPAD_DOWN:     -25 tps
 *  - X:             flip target sign (reverse direction)
 *  - Y:             zero target (stop)
 *  - A:             +kP (small step)
 *  - B:             -kP (small step)
 *  - DPAD_RIGHT:    +kD (small step)
 *  - DPAD_LEFT:     -kD (small step)
 *  - START:         toggle controller enabled/disabled (applies 0 power when disabled)
 *
 * Notes:
 *  - Target units are ticks/second (MotorEx default).
 *  - Make sure the motor name matches your Robot Configuration.
 */
@TeleOp(name = "PD Velocity Tester (FTCLib)", group = "Test")
public class TestPDVelocity extends LinearOpMode {

    // ==== USER SETTINGS ====
    private static final String MOTOR_NAME = "flywheel";  // <-- change to your config name
    private static final boolean INVERT_MOTOR = false;     // set true if velocity sign is reversed

    // Initial targets & gains (tune for your motor/gearbox/battery)
    private static final double INITIAL_TARGET_TPS = 1500.0; // ticks/sec
    private static final double INITIAL_kP = 0.0020;
    private static final double INITIAL_kD = 0.0008;

    // Optional feedforward (set non-zero if you want)
    private static final double INITIAL_kV = 0.0;   // power per ticks/sec
    private static final double INITIAL_kS = 0.0;   // static power

    // Output clamp
    private static final double MIN_POWER = -1.0;
    private static final double MAX_POWER =  1.0;

    // Bump sizes
    private static final double BUMP_BIG   = 100.0;  // tps
    private static final double BUMP_SMALL =  25.0;  // tps
    private static final double KP_STEP    = 0.0001;
    private static final double KD_STEP    = 0.00005;

    @Override
    public void runOpMode() {
        // Build FTCLib MotorEx
        MotorEx motor = new MotorEx(hardwareMap, MOTOR_NAME);
        motor.setInverted(INVERT_MOTOR);
        // We’ll run our own PD loop; raw power mode is fine. Velocity is still readable.
        try {
            motor.setRunMode(MotorEx.RunMode.RawPower);
        } catch (Throwable ignored) {
            // Older FTCLib versions may not have this enum; it’s OK to proceed.
        }

        // Controller
        FtclibVelocityPD controller = new FtclibVelocityPD(motor, INITIAL_kP, INITIAL_kD);
        controller.setFeedforward(INITIAL_kV, INITIAL_kS);
        controller.setOutputLimits(MIN_POWER, MAX_POWER);

        double targetTps = INITIAL_TARGET_TPS;
        controller.setTargetVelocity(targetTps);

        boolean enabled = true;

        // Simple edge-detection helpers
        boolean prevRB = false, prevLB = false, prevDU = false, prevDD = false;
        boolean prevX = false, prevY = false, prevStart = false;
        boolean prevA = false, prevB = false, prevDR = false, prevDL = false;

        waitForStart();

        while (opModeIsActive()) {
            // === Adjust target velocity ===
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;

            if (rb && !prevRB) targetTps += BUMP_BIG;
            if (lb && !prevLB) targetTps -= BUMP_BIG;
            if (du && !prevDU) targetTps += BUMP_SMALL;
            if (dd && !prevDD) targetTps -= BUMP_SMALL;

            // Flip or zero target
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            if (x && !prevX) targetTps = -targetTps;
            if (y && !prevY) targetTps = 0.0;

            // Toggle enabled/disabled
            boolean startBtn = gamepad1.start;
            if (startBtn && !prevStart) enabled = !enabled;

            // === Tune gains on the fly ===
            boolean btnA = gamepad1.a;
            boolean btnB = gamepad1.b;
            boolean dr = gamepad1.dpad_right;
            boolean dl = gamepad1.dpad_left;

            // Grab current gains from the controller by tracking our own copies
            // (PIDController doesn’t expose getters in older FTCLib; we’ll track manually)
            // We’ll maintain local copies and re-set when changed.
            // Start with the last we set:
            // (We’ll store them in fields so they persist)
            GainsHolder.udpateIfUnset(INITIAL_kP, INITIAL_kD); // one-time init

            if (btnA && !prevA) GainsHolder.kP += KP_STEP;
            if (btnB && !prevB) GainsHolder.kP = Math.max(0.0, GainsHolder.kP - KP_STEP);
            if (dr && !prevDR) GainsHolder.kD += KD_STEP;
            if (dl && !prevDL) GainsHolder.kD = Math.max(0.0, GainsHolder.kD - KD_STEP);

            controller.setGains(GainsHolder.kP, GainsHolder.kD);
            controller.setTargetVelocity(targetTps);

            // === Run control ===
            double applied;
            if (enabled) {
                applied = controller.computeAndApply();
            } else {
                motor.set(0.0);
                applied = 0.0;
            }

            // === Telemetry ===
            telemetry.addLine("PD Velocity Tester (FTCLib)");
            telemetry.addData("Enabled", enabled ? "YES" : "NO");
            telemetry.addData("Motor", MOTOR_NAME);
            telemetry.addData("Target (t/s)", "%.1f", targetTps);
            telemetry.addData("Measured (t/s)", "%.1f", controller.getMeasuredVelocity());
            telemetry.addData("Error (t/s)", "%.1f", (targetTps - controller.getMeasuredVelocity()));
            telemetry.addData("Power Applied", "%.3f", applied);
            telemetry.addLine();
            telemetry.addData("kP", "%.6f", GainsHolder.kP);
            telemetry.addData("kD", "%.6f", GainsHolder.kD);
            telemetry.addData("kV", "%.6f", INITIAL_kV);
            telemetry.addData("kS", "%.6f", INITIAL_kS);
            telemetry.addLine();
            telemetry.addLine("Bumps: RB/LB=±100  DpadUp/Down=±25  X=flip  Y=zero");
            telemetry.addLine("Gains: A=+kP  B=-kP  DpadRight=+kD  DpadLeft=-kD");
            telemetry.addLine("Start: enable/disable");
            telemetry.update();

            // Save button states for edge detection
            prevRB = rb; prevLB = lb; prevDU = du; prevDD = dd;
            prevX = x; prevY = y; prevStart = startBtn;
            prevA = btnA; prevB = btnB; prevDR = dr; prevDL = dl;

            // ~50 Hz loop
            sleep(20);
        }

        // Ensure motor is off when OpMode ends
        motor.set(0.0);
    }

    /** Tiny holder so we can keep/edit gains even if FTCLib PID doesn't expose getters. */
    private static class GainsHolder {
        private static boolean inited = false;
        static double kP, kD;
        static void udpateIfUnset(double p, double d) {
            if (!inited) { kP = p; kD = d; inited = true; }
        }
    }
}