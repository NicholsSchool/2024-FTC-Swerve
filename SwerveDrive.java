import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareDevice;

public class SwerveDrive {

    /* 
     * RED--------BLU
     *  |          |
     *  |          |
     *  |          |
     *  |          |
     * GRN--------YLW
     */

    private ServoImplEx redServo, blueServo, greenServo, yellowServo;
    private ServoImplEx[] servoList = new ServoImplEx[]{redServo, blueServo, greenServo, yellowServo};

    private IMU.Parameters imuParams;
    private IMU imu;

    private DcMotor redMotor, blueMotor, greenMotor, yellowMotor;
    private DcMotor[] motorList = new DcMotor[]{redMotor, blueMotor, greenMotor, yellowMotor};

    public void init() {

        HardwareMap hMap = new HardwareMap();

        imu = hMap.get(IMU.class, "imu");

        imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
           )
        );

        imu.initialize(imuParams);
        imu.resetYaw();

        redServo = hMap.get(ServoImplEx.class, "redServo");
        blueServo = hMap.get(ServoImplEx.class, "blueServo");
        greenServo = hMap.get(ServoImplEx.class, "greenServo");
        yellowServo = hMap.get(ServoImplEx.class, "yellowServo");

        for (ServoImplEx servo : servoList) {
            servo.setPwmEnable();
            servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
            servo.setDirection(Servo.Direction.FORWARD);
        }

        for (DcMotor motor : motorList){
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        }
    }
	
    public void resetYaw() {
        imu.resetYaw();
    }

    public void moveWithEncoder(double degreeAngle, double distance){
        //placeholder for later
        double meterToEncoder = 500;

        int position = ((int)degreeAngle + 180) / 360;
        int encoderTick = (int)distance * (int)meterToEncoder;
        
        //setting powers and positions
        for (DcMotor motor : motorList) {
            motor.setTargetPosition(encoderTick);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        for (ServoImplEx servo : servoList) {
           servo.setPosition(position);
        }

    }

    /**
     * Moves the Drivetrain with setPower()
     *
     * @strafeX Power for Left-Right Locomotion
     * @strafeY Power for Forward-Back Locomotion
     * @rotate The extent to which the drivetrain rotates
     */

	public void move(float strafeX, float strafeY, float rotate) {

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        
        float yaw = (float) (robotOrientation.getYaw(AngleUnit.RADIANS));

        //Calculations
		float theta1 = (float) Math.atan2(strafeX + rotate, strafeY + rotate) - yaw;
        float theta2 = (float) Math.atan2(strafeX - rotate, strafeY + rotate) - yaw;
        float power1 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) + Math.pow((strafeX + rotate) / 2, 2));
        float power2 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) + Math.pow((strafeX - rotate) / 2, 2));

        //Sending Powers and Angles to Motora

        //Servos
        for (ServoImplEx servo : servoList) {
            if (servo.equals(redServo) || servo.equals(yellowServo)) {
                servo.setPosition(thetaToServo(theta1));
            } else {
                servo.setPosition(thetaToServo(theta2));
            }
        }
        for (DcMotor motor : motorList) {
            if (motor.equals(redMotor) || motor.equals(yellowMotor)) {
                motor.setPower(power1);
            } else {
                motor.setPower(power2);
            }
        }
	}

    private float thetaToServo(float theta){
        float thetaInDegrees = (float) ((theta*180) / (Math.PI));
        //position of 1 means PWM of 2495 and 360-ish degree rotation
        //position of 0 means PWM of 505 and 0-ish degree rotation
        float degreesToServo = (thetaInDegrees + 180) / 360; // map range formula (simplified)
        return degreesToServo;
    }
}