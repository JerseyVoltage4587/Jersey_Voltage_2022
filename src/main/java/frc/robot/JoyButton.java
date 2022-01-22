package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class JoyButton extends Button {
	private JoyDir joyDir;
	private Joystick stick;
	private int axis;
		
	public enum JoyDir {
		DOWN,UP;
	}
		
	/**
	 * Uses Joystick as  a button input.
	 * @param joystick
	 * @param direction
	 * @param axis
	 */
	public JoyButton(Joystick joystick, JoyDir direction, int axis) {
		joyDir = direction;
		stick = joystick;
		this.axis = axis;
	}

	public double getRawAxis() {
		return stick.getRawAxis(axis);
	}

	@Override
	public boolean get() {
		switch(joyDir) {
			case UP:
				if(stick.getRawAxis(axis) <= -.1) {
					return true;
				}
				break;
			case DOWN:
				if(stick.getRawAxis(axis) >= .1) {
					return true;
				}
				break;
		}
		return false;
	}
}
