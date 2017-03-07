package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mechanum", group = "FireBot")
public class FireWires_Mechanum extends OpMode {
    private static final float INTAKE_POWER = 1;
    private static final float INTAKE_POWER_REVERSE = -1;
    private static final float JOYSTICK_DEADBAND = .2f;
    private static final float JOYSTICK_OFFSET = 0;
    private static final float JOYSTICK_GAIN = .2f;
    private static final float SHOOTER_SERVO_UP = -1;
    private static final float SHOOTER_SERVO_DOWN = 1;
    private static final float DRIVE_SPEED = 1f;

    /* Declare OpMode members. *///
    /**
     * To test a push for the final time
     */
    HardwareFireWiresBot robot = new HardwareFireWiresBot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INITF
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Systems Initialized...");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        float left2;
        float right2;
        left2 = gamepad2.left_stick_y;
        right2 = gamepad2.right_stick_y;

        //Create and set values to used variables
        double speed = 0, angle = 0, dchange = 0;
        double retValues [];

        //Instance of cartesianToPolar method used for changing the cartesian values into polar values.
        retValues = cartesianToPolar(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        //Set retValues array to shown values. retValues is used to return the variables used in multiple methods.
        speed = retValues[0];
        angle = retValues[1];
        dchange = retValues[2];

        //Instance of polarToWheelSpeed method used for powering wheels
        polarToWheelSpeed(speed, angle, dchange);

        /**
         * Turn intake on at 100% to fix stuck balls
         */
        if (gamepad2.right_trigger == 1) {
            robot.intake(INTAKE_POWER_REVERSE);
        }

        /**
         * Turn intake on at 100% for normal intake
         */
        if (gamepad2.left_trigger == 1) {
            robot.intake(INTAKE_POWER);
        }

        /**
         * Turn intake off if bumpers not pressed
         */
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            robot.intake(0);
        }

        if (!gamepad1.x & gamepad1.b) {
            robot.pusherServo.setPosition(.3);
        }

        if (gamepad1.x & !gamepad1.b) {
            robot.pusherServo.setPosition(.7);
        }

        if (!gamepad1.x & !gamepad1.b) {
            robot.pusherServo.setPosition(0);
        }

        /**
         * Fire
         */
        if (gamepad2.a || gamepad2.x) {
            if (gamepad2.a) {
                robot.fire();
            } else {
                long setTime = System.currentTimeMillis();
                robot.fireFarther();

                /* Wait 1 second for the ball to settle */
                if (System.currentTimeMillis() - setTime > 1000) {
                    /* FIRE */
                    robot.shootServo.setPosition(SHOOTER_SERVO_UP);
                }
            }
        } else {
            say("Stopping Fire");
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
        }

        if (gamepad2.b && !gamepad2.a && left2 == 0) {
            robot.move_shoot_servo(SHOOTER_SERVO_UP);
        }
        if (!gamepad2.b && !gamepad2.a && left2 == 0) {
            robot.move_shoot_servo(SHOOTER_SERVO_DOWN);
            telemetry.addData("Say", "Servo Down");
        } else {
            telemetry.addData("Say", left2);
        }

        if (right2 != 0) {
            robot.liftServo.setPosition(right2);
            say("Right2: " + right2);
        } else {
            robot.liftServo.setPosition(1);
        }

        if (left2 != 0) {
            robot.shootServo.setPosition(left2);
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void print(String command, String string) {
        telemetry.addData(command, string);
    }

    private void say(String string) {
        telemetry.addData("Say", string);
    }

    public double[] cartesianToPolar(double y1, double x1, double x2) {

        //Reset retValues to 0 for later use
        double[] retValues = new double[]{0.0, 0.0, 0.0};
        double speed = 0.0, angle = 0.0, dchange = 0.0;

        //Change joypad values into useful polar values
        speed = Math.sqrt((y1 * y1) + (x1 * x1));
        angle = Math.atan2(x1, -y1);
        dchange = -x2 / 3.33;

        //Add polar values to retValues array
        retValues[0] = speed;
        retValues[1] = angle;
        retValues[2] = dchange;
        return retValues;
    }

    public void polarToWheelSpeed(double speed, double angle, double dchange) {

        //Create Variables used only in this method
        double pos1, pos2, pos3, pos4, maxValue;

        //Define unscaled voltage multipliers
        pos1 = speed * Math.sin(angle + (Math.PI / 4)) + dchange;
        pos2 = speed * Math.cos(angle + (Math.PI / 4)) - dchange;
        pos3 = speed * Math.cos(angle + (Math.PI / 4)) + dchange;
        pos4 = speed * Math.sin(angle + (Math.PI / 4)) - dchange;

        //VOLTAGE MULTIPLIER SCALER

        //Set maxValue to pos1 absolute
        maxValue = Math.abs(pos1);

        //If pos2 absolute is greater than maxValue, then make maxValue equal to pos2 absolute
        if (Math.abs(pos2) > maxValue) {
            maxValue = Math.abs(pos2);
        }

        //If pos3 absolute is greater than maxValue, then make maxValue equal to pos3 absolute
        if (Math.abs(pos3) > maxValue) {
            maxValue = Math.abs(pos3);
        }

        //If pos4 absolute is greater than maxValue, then make maxValue equal to pos4 absolute
        if (Math.abs(pos4) > maxValue) {
            maxValue = Math.abs(pos4);
        }

        //Check if need to scale -- if not set to 1 to nullify scale
        if (maxValue <= 1) {
            maxValue = 1;
        }

        //Power motors with scaled voltage multipliers
        robot.leftFront.setPower(pos1 / maxValue);
        robot.rightFront.setPower(pos2 / maxValue);
        robot.leftMotor.setPower(pos3 / maxValue);
        robot.rightMotor.setPower(pos4 / maxValue);

    }
}
