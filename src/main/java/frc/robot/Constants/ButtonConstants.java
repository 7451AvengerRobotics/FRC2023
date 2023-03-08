package frc.robot.Constants;

public class ButtonConstants {

  public static final int CONTROLLER_PORT = 0;
  public static final int BUTTON_PANEL_PORT = 1;
  /*
   * Button Panel Mapping
   * ---------------------------
   * |          | 11 | 12|     | 
   * |    ^    | 1 | 2 | 3 | 4 |
   * |   < >   | 5 | 6 | 7 | 8 |
   * |    v    | 9 | 10 |      |   
   * ---------------------------
   */

  // All Constants are subject to change once we figure out what each one does in the old code. 
  public static final int TURN_TURRET_LEFT = 1;
  public static final int TURN_TURRET_RIGHT = 2;

  public static final int ARM_EXTEND = 3;
  public static final int ARM_RETRACT = 4;

  public static final int CLAW_IN = 5;
  public static final int CLAW_OUT = 6;

  public static final int CLAW_EXTEND = 9;
  public static final int CLAW_RETRACT = 10;

  public static final int VBAR_UP = 7;
  public static final int VBAR_DOWN = 8;
   
}