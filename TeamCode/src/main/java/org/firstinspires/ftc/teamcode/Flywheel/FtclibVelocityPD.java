package org.firstinspires.ftc.teamcode.Flywheel;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * FTCLib-only PD velocity controller (no RobotCore imports).
 * - Feedback: MotorEx.getVelocity()  (ticks/sec by default)
 * - Output:   MotorEx.set(power)     ([-1, 1] by default)
 * - I term is fixed at 0 -> PD behavior
 */
public class FtclibVelocityPD {

    private final MotorEx motor;
    private final PIDController pd;   // (kP, 0, kD)

    // Optional feedforward
    private double kV = 0.0;   // per (ticks/sec)
    private double kS = 0.0;   // static

    private double targetVel = 0.0;   // ticks/sec
    private double minOut = -1.0;
    private double maxOut =  1.0;

    private double lastApplied = 0.0;

    /**
     * @param motor a configured FTCLib MotorEx (created elsewhere)
     * @param kP    proportional gain
     * @param kD    derivative gain
     */
    public FtclibVelocityPD(final MotorEx motor, double kP, double kD) {
        this.motor = motor;
        this.pd = new PIDController(kP, 0.0, kD); // I=0 -> PD
        // Use whatever run mode your MotorEx is already in; caller controls that.
        // You may call motor.setInverted(...) outside, if needed.
    }

    /** Update PD gains (I stays 0). */
    public void setGains(double kP, double kD) {
        pd.setPID(kP, 0.0, kD);
    }

    /** Optional feedforward: kV * target + kS * sign(target). */
    public void setFeedforward(double kV, double kS) {
        this.kV = kV;
        this.kS = kS;
    }

    /** Clamp output power. */
    public void setOutputLimits(double min, double max) {
        this.minOut = Math.min(min, max);
        this.maxOut = Math.max(min, max);
    }

    /** Target velocity in ticks/sec. */
    public void setTargetVelocity(double targetTicksPerSec) {
        this.targetVel = targetTicksPerSec;
    }

    public double getTargetVelocity() { return targetVel; }

    /** Current measured velocity (ticks/sec). */
    public double getMeasuredVelocity() {
        return motor.getVelocity();
    }

    /** Compute PD (+FF) output, clamped, without applying to the motor. */
    public double compute() {
        double measured = getMeasuredVelocity();
        double out = pd.calculate(measured, targetVel);

        if (kV != 0.0 || kS != 0.0) {
            out += kV * targetVel + kS * Math.signum(targetVel);
        }

        if (out > maxOut) out = maxOut;
        if (out < minOut) out = minOut;

        return out;
    }

    /** Compute and apply to the motor. Returns the applied power. */
    public double computeAndApply() {
        lastApplied = compute();
        motor.set(lastApplied);
        return lastApplied;
    }

    /** Last power applied by computeAndApply(). */
    public double getLastAppliedPower() {
        return lastApplied;
    }
}
