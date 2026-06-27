package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * ============================================================================
 * GKMecanumTest
 * ----------------------------------------------------------------------------
 * Εκπαιδευτικό παράδειγμα οδήγησης Mecanum για FTC.
 *
 * Το αρχείο αυτό δείχνει:
 * 1. Πώς γίνεται η αρχικοποίηση των κινητήρων.
 * 2. Πώς ρυθμίζεται η φορά περιστροφής τους.
 * 3. Πώς χρησιμοποιείται η IMU.
 * 4. Πώς υλοποιείται Robot Relative Drive.
 * 5. Πώς υλοποιείται Field Relative Drive.
 * 6. Πώς υπολογίζεται η ισχύς κάθε mecanum τροχού.
 *
 * Σημείωση:
 * Ο κώδικας είναι ο ίδιος με τον αρχικό. Έχουν προστεθεί μόνο
 * εκπαιδευτικά σχόλια στα Ελληνικά.
 * ============================================================================
 */
@TeleOp(name = "Robot: Gavriil Karpathios Mecanum Drive Test", group = "Tests")
public class GKMecanumTest extends OpMode {

    // =========================================================================
    // Δήλωση κινητήρων
    // Κάθε μεταβλητή αντιστοιχεί σε έναν φυσικό κινητήρα του robot.
    // =========================================================================
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    // =========================================================================
    // IMU (Inertial Measurement Unit)
    // Χρησιμοποιείται για να γνωρίζει το πρόγραμμα τον προσανατολισμό
    // (Yaw, Pitch, Roll) του robot.
    // =========================================================================
    private IMU imu;

    // Μέγιστη επιτρεπόμενη ταχύτητα.
    private static final double MAX_SPEED = 1.0;

    @Override
    public void init() {

        // Αντιστοίχιση μεταβλητών με τις συσκευές του Robot Configuration.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Ρύθμιση φοράς περιστροφής των κινητήρων.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Χρήση encoders για ακριβέστερο έλεγχο.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Φρενάρισμα όταν η ισχύς γίνει μηδέν.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Αρχικοποίηση IMU.
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {

        telemetry.addLine("Το αριστερό joystick ελέγχει την κίνηση.");
        telemetry.addLine("Το δεξί joystick περιστρέφει το robot.");

        // Robot Relative Drive
        drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }

    /**
     * Field Relative Drive
     *
     * Ο οδηγός κινεί το robot ως προς το γήπεδο και όχι ως προς
     * τον προσανατολισμό του robot.
     */
    private void driveFieldRelative(double forward, double right, double rotate) {

        // Μετατροπή του διανύσματος του joystick σε πολικές συντεταγμένες.
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Αφαίρεση της γωνίας του robot από την IMU.
        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        // Μετατροπή ξανά σε καρτεσιανές συντεταγμένες.
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    /**
     * Robot Relative Mecanum Drive
     *
     * Συνδυάζει:
     * - Κίνηση εμπρός / πίσω
     * - Πλάγια ολίσθηση (Strafe)
     * - Περιστροφή
     *
     * και υπολογίζει την τελική ισχύ κάθε κινητήρα.
     */
    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Rotate", rotate);

        double maxPower = 1.0;

        // Κανονικοποίηση ώστε καμία τιμή να μην ξεπερνά το [-1,1].
        maxPower = Math.max(MAX_SPEED, Math.abs(frontLeftPower));
        maxPower = Math.max(MAX_SPEED, Math.abs(frontRightPower));
        maxPower = Math.max(MAX_SPEED, Math.abs(backRightPower));
        maxPower = Math.max(MAX_SPEED, Math.abs(backLeftPower));

        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);

        // Αποστολή ισχύος στους κινητήρες.
        frontLeftDrive.setPower(MAX_SPEED * (frontLeftPower / maxPower));
        frontRightDrive.setPower(MAX_SPEED * (frontRightPower / maxPower));
        backLeftDrive.setPower(MAX_SPEED * (backLeftPower / maxPower));
        backRightDrive.setPower(MAX_SPEED * (backRightPower / maxPower));
    }
}
