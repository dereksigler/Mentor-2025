package org.firstinspires.ftc.teamcode.Flywheel;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class flywheelVelocity {

    private DcMotor flywheelSpin;

    public void init(HardwareMap hwMap) {
        flywheelSpin = hwMap.get(DcMotor.class, "flywheel");
        flywheelSpin.setDirection(DcMotorSimple.Direction.REVERSE);

        // Need to add encoders!

    }

//  Review  https://docs.ftclib.org/ftclib/features/controllers
//    to implement

}

