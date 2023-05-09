import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveDriver {
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

    HardwareMap hMap = new HardwareMap();

    public void init() {
        redServo = hMap.get(ServoImplEx.class, "redServo");
        blueServo = hMap.get(ServoImplEx.class, "blueServo");
        greenServo = hMap.get(ServoImplEx.class, "greenServo");
        yellowServo = hMap.get(ServoImplEx.class, "yellowServo");
        for (ServoImplEx servo : servoList) {

        }

        for (DcMotor motor : motorList){
            
        }
    }
	
	public void move(float strafeX, float strafeY, float rotate) {
		float theta1 = (float) Math.atan2(strafeY + rotate, strafeX + rotate);
        float theta2 = (float) Math.atan2(strafeY + rotate, strafeX - rotate);
        float power1 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) +Math.pow((strafeX + rotate) / 2, 2));
        float power2 = (float) Math.sqrt(Math.pow((strafeY + rotate) / 2, 2) +Math.pow((strafeX - rotate) / 2, 2));
	}

    private float thetatoEncoder(float theta){
        float thetaInDegrees = (float) ((theta*180) / (Math.PI));
    }
}