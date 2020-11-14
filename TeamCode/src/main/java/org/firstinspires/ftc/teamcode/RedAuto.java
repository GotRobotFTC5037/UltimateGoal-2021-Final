package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto", group = "Pushbot")
public class RedAuto extends LinearOpMode {
    HardwarePerseverence robot           = new HardwarePerseverence();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation angles;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double DIST_PER_REV = (4 * 2.54 * Math.PI) / COUNTS_PER_MOTOR_REV;
    public double subtractAngle(double angleA,
                                double angleB) {
        double result;
        result = angleA - angleB;
        if (result > 180) {
            result = result - 360;
        }
        return result;
    }
    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }
    public void autoPilot(double heading,
                           double pose,
                           double distance,
                           double power,
                           double timeoutS) {

        double currentHeading;
        double drvPower = power;
        double adjPower;
        double currentDistance;
        double driveAngle;
        double headingRadians;
        double poseDegrees;
        double distX = 0;
        double distY = 0;
        double dY;
        double dX;
        double oldLeft = robot.leftDrive.getCurrentPosition();
        double oldRight = robot.rightDrive.getCurrentPosition();
        double oldBackLeft = robot.leftBackDrive.getCurrentPosition();
        double oldBackRight = robot.rightBackDrive.getCurrentPosition();
        double newLeft = 0;
        double newRight = 0;
        double newBackLeft = 0;
        double newBackRight = 0;
        runtime.reset();
        while ((runtime.seconds() < timeoutS)) {
            currentHeading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            headingRadians = ((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416);
           // headingRadians = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            poseDegrees = ((pose - 3.1416 / 2) % (2 * 3.1416)) * (360 / (2 * 3.1416));
            if (poseDegrees > 180) {
                poseDegrees -= 360;
            }
            if (poseDegrees < -180) {
                poseDegrees += 360;
            }
            driveAngle = subtractAngle(poseDegrees, currentHeading);
            adjPower = subtractAngle(poseDegrees, currentHeading) / 120;
            newLeft = robot.leftDrive.getCurrentPosition();
            newRight = robot.rightDrive.getCurrentPosition();
            newBackRight = robot.rightBackDrive.getCurrentPosition();
            newBackLeft = robot.leftBackDrive.getCurrentPosition();

            dX = ((((newLeft - oldLeft) - (newBackLeft - oldBackLeft)
                    - (newRight - oldRight) + (newBackRight - oldBackRight)) * Math.sin(Math.PI / 4)) / 4.0) * DIST_PER_REV;
            dY = (((newLeft - oldLeft) + (newBackLeft - oldBackLeft)
                    + (newRight - oldRight) + (newBackRight - oldBackRight)) / 4.0) * DIST_PER_REV;
            distX += (Math.sin(headingRadians) * dX + Math.cos(headingRadians) * dY);
            distY += (Math.sin(headingRadians) * dY + Math.cos(headingRadians) * dX);
            oldLeft = newLeft;
            oldRight = newRight;
            oldBackLeft = newBackLeft;
            oldBackRight = newBackRight;

            currentDistance = Math.sqrt(distX * distX + distY * distY);
            if (currentDistance > distance) {
                robot.leftDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                return;
            }

            driveAngle = ((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416) + heading;
//            driveAngle = 0.5*Math.PI;
//            adjPower = 0;

            double robotAngle = driveAngle - Math.PI / 4;
            final double v1 = power * Math.cos(robotAngle) - adjPower;
            final double v2 = power * Math.sin(robotAngle) + adjPower;
            final double v3 = power * Math.sin(robotAngle) - adjPower;
            final double v4 = power * Math.cos(robotAngle) + adjPower;


            robot.leftDrive.setPower(v1);
            robot.rightDrive.setPower(v2);
            robot.leftBackDrive.setPower(v3);
            robot.rightBackDrive.setPower(v4);
        }
    }
    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitMilis(50);


    }
}
