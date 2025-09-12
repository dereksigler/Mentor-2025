package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;

    public void init(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hwMap.get(DcMotor.class, "backRightDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
            double frontLeftPower = forward + strafe + rotate;
            double backLeftPower = forward - strafe + rotate;
            double frontRightPower = forward - strafe - rotate;
            double backRightPower = forward + strafe - rotate;

            double maxPower = 1.0;
            double maxSpeed = 1.0;  // lower for guests

            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
            backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
            frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
            backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
        }
        // The above is simply robot centric drive
        // Below we apply the IMU for field centric

    public void driveFieldCentric(double forward, double strafe, double rotate){
            double theta = Math.atan2(forward,strafe);
            double r = Math.hypot(strafe,forward);

            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double newForward = r * Math.sin(theta);
            double newStrafe = r * Math.cos(theta);

            this.drive(newForward,newStrafe,rotate);
        }

}
