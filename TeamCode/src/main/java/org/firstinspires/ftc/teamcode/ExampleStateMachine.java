package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Simple State Machine Example")
public class ExampleStateMachine extends LinearOpMode {

    // Define the 7 states
    enum State { STEP1, STEP2, STEP3, STEP4,  STEP5, STEP6, STEP7, DONE }

    private State currentState = State.STEP1;

    @Override
    public void runOpMode() {
        telemetry.addLine("Ready to run state machine.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && currentState != State.DONE) {
            switch (currentState) {
                case STEP1:
                    telemetry.addLine("Running Step 1...");
                    // Do something here (e.g., drive forward)
                    sleep(1000); // simulate action
                    currentState = State.STEP2;
                    break;

                case STEP2:
                    telemetry.addLine("Running Step 2...");
                    // Do something else
                    sleep(1000);
                    currentState = State.STEP3;
                    break;

                case STEP3:
                    telemetry.addLine("Running Step 3...");
                    sleep(1000);
                    currentState = State.STEP4;
                    break;

                case STEP4:
                    telemetry.addLine("Running Step 4...");
                    sleep(1000);
                    currentState = State.STEP5;
                    break;

                case STEP5:
                    telemetry.addLine("Running Step 5...");
                    sleep(1000);
                    currentState = State.STEP6;
                    break;

                case STEP6:
                    telemetry.addLine("Running Step 6...");
                    sleep(1000);
                    currentState = State.STEP7;
                    break;

                case STEP7:
                    telemetry.addLine("Running Step 7...");
                    sleep(1000);
                    currentState = State.DONE;
                    break;
            }

            telemetry.addData("Current State", currentState);
            telemetry.update();
        }

        telemetry.addLine("State Machine Finished.");
        telemetry.update();
    }
}
