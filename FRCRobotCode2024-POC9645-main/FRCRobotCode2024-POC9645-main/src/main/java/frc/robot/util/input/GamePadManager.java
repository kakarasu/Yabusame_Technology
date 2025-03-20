package frc.robot.util.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class GamePadManager extends GenericHID{


    private final byte type;
    private final int port;
    public GamePadManager(XboxController controller) {
        super(controller.getPort());
        type = ControllerVariables.TYPE_XBOX;
        port = controller.getPort();

    }
    public GamePadManager(PS4Controller controller) {
        super(controller.getPort());
        type = ControllerVariables.TYPE_DUALSHOCK_4;
        port = controller.getPort();
    }

    public double getLeftX(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kLeftX.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kLeftX.value);
            default -> 0;
        };
    }

    public double getLeftY(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kLeftY.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kLeftY.value);
            default -> 0;
        };
    }

    public double getRightX(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kRightX.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kRightX.value);
            default -> 0;
        };
    }

    public double getRightY(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kRightY.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kRightY.value);
            default -> 0;
        };
    }

    public double getLeftTriggerAxis(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kLeftTrigger.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kL2.value);
            default -> 0;
        };
    }

    public double getRightTriggerAxis(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawAxis(XboxController.Axis.kRightTrigger.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawAxis(PS4Controller.Axis.kR2.value);
            default -> 0;
        };
    }

    public boolean getLeftBumper(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kLeftBumper.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kL1.value);
            default -> false;
        };
    }

    public boolean getRightBumper(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kRightBumper.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kR1.value);
            default -> false;
        };
    }
    public boolean getLeftBumperPressed(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kLeftBumper.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kL1.value);
            default -> false;
        };
    }

    public boolean getRightBumperPressed(){
        return switch (type) {
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kRightBumper.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kR1.value);
            default -> false;
        };
    }

    public BooleanEvent leftBumper(EventLoop loop){
        return new BooleanEvent(loop,this::getLeftBumper);
    }

    public BooleanEvent rightBumper(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightBumper);
    }

    public boolean getLeftStickButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kLeftStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kL3.value);
            default -> false;
        };
    }

    public boolean getRightStickButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kRightStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kR3.value);
            default -> false;
        };
    }

    public boolean getLeftStickButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kLeftStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kL3.value);
            default -> false;
        };
    }

    public boolean getRightStickButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kRightStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kR3.value);
            default -> false;
        };
    }

    public boolean getLeftStickButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kLeftStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kL3.value);
            default -> false;
        };
    }

    public boolean getRightStickReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kRightStick.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kR3.value);
            default -> false;
        };
    }

    public BooleanEvent leftStickButton(EventLoop loop){
        return new BooleanEvent(loop,this::getLeftStickButton);
    }

    public BooleanEvent rightStickButton(EventLoop loop){
        return new BooleanEvent(loop,this::getRightStickButton);
    }

    public boolean getPadRightButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kB.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kCircle.value);
            default -> false;
        };
    }

    public boolean getPadRightButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kB.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kCircle.value);
            default -> false;
        };
    }

    public boolean getPadRightButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kB.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kCircle.value);
            default -> false;
        };
    }

    public BooleanEvent padRightButton(EventLoop loop){
        return new BooleanEvent(loop,this::getPadRightButton);
    }

    public boolean getPadDownButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kA.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kCross.value);
            default -> false;
        };
    }

    public boolean getPadDownButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kA.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kCross.value);
            default -> false;
        };
    }

    public boolean getPadDownButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kA.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kCross.value);
            default -> false;
        };
    }

    public BooleanEvent padDownButton(EventLoop loop){
        return new BooleanEvent(loop,this::getPadDownButton);
    }

    public boolean getPadLeftButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kX.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kSquare.value);
            default -> false;
        };
    }

    public boolean getPadLeftButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kX.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kSquare.value);
            default -> false;
        };
    }

    public boolean getPadLeftButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kX.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kSquare.value);
            default -> false;
        };
    }

    public BooleanEvent padLeftButton(EventLoop loop){
        return new BooleanEvent(loop,this::getPadLeftButton);
    }

    public boolean getPadUpButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kY.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButton(PS4Controller.Button.kTriangle.value);
            default -> false;
        };
    }

    public boolean getPadUpButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kY.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonPressed(PS4Controller.Button.kTriangle.value);
            default -> false;
        };
    }

    public boolean getPadUpButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kY.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> this.getRawButtonReleased(PS4Controller.Button.kTriangle.value);
            default -> false;
        };
    }

    public BooleanEvent padUpButton(EventLoop loop){
        return new BooleanEvent(loop,this::getPadUpButton);
    }

    public boolean getSmallLeftButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kBack.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButton(PS4Controller.Button.kShare.value);
            default -> false;
        };
    }

    public boolean getSmallLeftButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kBack.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButtonPressed(PS4Controller.Button.kShare.value);
            default -> false;
        };
    }

    public boolean getSmallLeftButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kBack.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButtonReleased(PS4Controller.Button.kShare.value);
            default -> false;
        };
    }

    public BooleanEvent smallLeftButton(EventLoop loop){
        return new BooleanEvent(loop,this::getSmallLeftButton);
    }

    public boolean getSmallRightButton(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButton(XboxController.Button.kStart.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButton(PS4Controller.Button.kOptions.value);
            default -> false;
        };
    }

    public boolean getSmallRightButtonPressed(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonPressed(XboxController.Button.kStart.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButtonPressed(PS4Controller.Button.kOptions.value);
            default -> false;
        };
    }

    public boolean getSmallRightButtonReleased(){
        return switch (type){
            case ControllerVariables.TYPE_XBOX -> this.getRawButtonReleased(XboxController.Button.kStart.value);
            case ControllerVariables.TYPE_DUALSHOCK_4 -> getRawButtonReleased(PS4Controller.Button.kOptions.value);
            default -> false;
        };
    }

    public BooleanEvent smallRightButton(EventLoop loop){
        return new BooleanEvent(loop,this::getSmallRightButton);
    }

    public BooleanEvent leftTrigger(EventLoop loop, double threshold){
        return new BooleanEvent(loop,()-> this.getLeftTriggerAxis() > threshold);
    }

    public BooleanEvent leftTrigger(EventLoop loop){
        return leftTrigger(loop,0.5);
    }

    public BooleanEvent rightTrigger(EventLoop loop,double threshold){
        return new BooleanEvent(loop,()-> this.getLeftTriggerAxis() > threshold);
    }

    public BooleanEvent rightTrigger(EventLoop loop){
        return rightTrigger(loop,0.5);
    }

    public boolean getTouchpad(){
        return type == ControllerVariables.TYPE_DUALSHOCK_4 && getRawButton(PS4Controller.Button.kTouchpad.value);
    }

    public boolean getTouchpadPressed(){
        return type == ControllerVariables.TYPE_DUALSHOCK_4 && getRawButtonPressed(PS4Controller.Button.kTouchpad.value);
    }
    public boolean getTouchpadReleased(){
        return type == ControllerVariables.TYPE_DUALSHOCK_4 && getRawButtonReleased(PS4Controller.Button.kTouchpad.value);
    }

    public BooleanEvent touchpad(EventLoop loop){
        return new BooleanEvent(loop,this::getTouchpad);
    }


}
