import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class SwerveDriver {

    /* 
     * RED--------BLU
     *  |          |
     *  |          |
     *  |          |
     *  |          |
     * GRN--------YLW
     */

    ServoImplEx redServo;
	ServoImplEx blueServo;
	ServoImplEx greenServo;
	ServoImplEx yellowServo;
    ServoImplEx[] servoList = new ServoImplEx[]{redServo, blueServo, greenServo, yellowServo};
    

    DcMotor redMotor;
    DcMotor blueMotor;
    DcMotor greenMotor;
    DcMotor yellowMotor;
    DcMotor[] motorList = new DcMotor[]{redMotor, blueMotor, greenMotor, yellowMotor};

    public void init() {

        HardwareMap hMap = new HardwareMap();

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
	
	public void move(float strafeX, float strafeY, float rotate) {

        //Calculations
		float theta1 = (float) Math.atan2(strafeY + rotate, strafeX + rotate);
        float theta2 = (float) Math.atan2(strafeY + rotate, strafeX - rotate);
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
        float degreesToEncoderPos = thetaInDegrees/360;
        return degreesToEncoderPos;
    }
}