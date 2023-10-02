import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

class SwerveTeleop extends OpMode {

    SwerveDrive sd = new SwerveDrive();

    @Override
    public void init() {
        sd.init();
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

        float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;

        sd.move(x1, y1, x2);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

