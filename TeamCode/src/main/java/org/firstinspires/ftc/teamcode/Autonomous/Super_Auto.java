package org.firstinspires.ftc.teamcode.Autonomous;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareFireWiresBot;

@Autonomous(name = "Super Auto", group = "FireBot")
public class Super_Auto extends LinearOpMode {
    HardwareFireWiresBot robot = new HardwareFireWiresBot();
    long start_time;
    private ElapsedTime runtime = new ElapsedTime();
    static final double SHOOTER_POWER = .25;
    private String particlePref;
    private String beaconPref;
    private String capBallPref;
    private String parkingPref;
    private String alliance;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.30;
    static final double TURN_SPEED = 0.25;

    static final double FLOOR_REFLECTANCE = 0.2;
    static final double LINE_REFLECTANCE = 0.04;
    static final double THRESHOLD_REFLECTANCE = (LINE_REFLECTANCE + FLOOR_REFLECTANCE) / 2;
    static final double COLOR_THRESHOLD = 2;
    double reflectance = 0.0;

    static int bluecolor;
    static int redcolor;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.color.enableLed(false);
        robot.ods.enableLed(true);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        getAutonomousPrefs();

        telemetry.addLine("Particles: " + particlePref);
        telemetry.addLine("Beacons: " + beaconPref);
        telemetry.addLine("Cap Ball: " + capBallPref);
        telemetry.addLine("Parking: " + parkingPref);
        telemetry.addLine("Alliance: " + alliance);
        telemetry.update();
        //Initial stuff

        updateColors();
        switch (alliance) {
            case "Blue Alliance":
                while (opModeIsActive()) {
                    if (!particlePref.equalsIgnoreCase(("Do not shoot any particles"))) {
                    }
                    if (!beaconPref.equalsIgnoreCase("Do not activate any beacons")) {
                        encoderDrive(DRIVE_SPEED, 8, 8, 6);      /* Drive forward */
                        encoderDrive(TURN_SPEED, 3.5, -1.5, 2.0);
                        encoderDrive(DRIVE_SPEED, 6, 6, 6);      /* Drive forward */
                        bumpBeaconBlue();
                    }
                }



                break;
            case "Red Alliance":
                encoderDrive(DRIVE_SPEED, 10, 10, 6.0);      /* Drive forward */
                sleep(1000);
                encoderDrive(TURN_SPEED, -2, 4, 2.0);
                sleep(1000);
                encoderDrive(DRIVE_SPEED,  10, 10, 2.0);      /* Drive forward */
                telemetry.addData("Red", robot.color.red());
                telemetry.addData("Blue", robot.color.blue());
                telemetry.update();
                while (robot.color.red() < COLOR_THRESHOLD) {
                    robot.leftMotor.setPower((DRIVE_SPEED * 2));
                    robot.rightMotor.setPower((DRIVE_SPEED * 2));
                    sleep(1000);
                    telemetry.addData("Blue", robot.color.blue());
                    telemetry.addData("Red", robot.color.red());
                    telemetry.update();
                    robot.leftMotor.setPower(-DRIVE_SPEED);
                    robot.rightMotor.setPower(-DRIVE_SPEED);
                    sleep(500);
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    sleep(6000);
                }
                break;

        }

        // Stop all motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }

    public void shoot2(){
        robot.leftShooter.setPower(SHOOTER_POWER);
        robot.rightShooter.setPower(SHOOTER_POWER);
        sleep(2000);
        robot.shootServo.setPosition(-1);
        sleep(500);
        robot.shootServo.setPosition(1);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        sleep(1000);
        /* Fire again! */
        robot.shootServo.setPosition(1);
        sleep(2000);
        robot.leftShooter.setPower(SHOOTER_POWER);
        robot.rightShooter.setPower(SHOOTER_POWER);
        sleep(1000);
        robot.shootServo.setPosition(-1);
        sleep(1000);
    }
    private void getAutonomousPrefs()
    {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        particlePref = preferences.getString("How Many Particles Should We Shoot?", "");
        beaconPref = preferences.getString("Which beacons should we activate?", "");
        capBallPref = preferences.getString("Should we bump the cap ball off the center vortex?", "");
        parkingPref = preferences.getString("Where should we park?", "");
        alliance = preferences.getString("Which alliance are we on?", "");
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        leftInches = -leftInches;
        rightInches = -rightInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }//

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public int getRedColor(){
        return robot.color.red();
    }
    public int getBlueColor(){
        return robot.color.blue();
    }
    public  void updateColors(){
        bluecolor = robot.color.blue();
        redcolor = robot.color.red();
    }
    public void hitBeacon(){
        while(opModeIsActive()) {
            sleep(500);
            bluecolor = robot.color.blue();
            telemetry.addData("Red", bluecolor);
            telemetry.addData("Blue", robot.color.blue());
            telemetry.update();
            Robot_Methods.driveForSecondsAtPower(robot, -DRIVE_SPEED, .5);
            sleep(500);
            if(bluecolor > 0){
                telemetry.addData("Found!", robot.color.blue());
                telemetry.update();
                Robot_Methods.driveForSecondsAtPower(robot, DRIVE_SPEED, 1);
                sleep(3000);
                break;
            } else {
                sleep(300);
                bluecolor = robot.color.blue();
                if(robot.color.blue() > 0){
                    telemetry.addData("Found!", robot.color.blue());
                    telemetry.update();
                    Robot_Methods.driveForSecondsAtPower(robot, DRIVE_SPEED, 1);
                    break;
                } else {
                    telemetry.addData("Not Found!", robot.color.blue());
                    telemetry.update();
                }

            }
            updateColors();
            //reverse after hitting
        }
    }

    public void bumpBeaconBlue() {
        while (opModeIsActive() && robot.color.blue() < COLOR_THRESHOLD) {
            telemetry.addData("Bump Beacon", "Not Found: " + robot.color.blue());
            telemetry.update();
            Robot_Methods.driveForSecondsAtPower(robot, -DRIVE_SPEED, .3); /* Drive Forward 300ms */
            Robot_Methods.driveForSecondsAtPower(robot, DRIVE_SPEED, .3);  /* Drive Back 300ms */
            sleep(1000);
            telemetry.addData("Bump Beacon", "Found: " + robot.color.blue());
            telemetry.update();
        }
    }
}
