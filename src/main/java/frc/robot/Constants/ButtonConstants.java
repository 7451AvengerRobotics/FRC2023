package frc.robot.Constants;

public class ButtonConstants {

  public static final int CONTROLLER_PORT = 0;
  public static final int BUTTON_PANEL_PORT = 1;
  /*
   * Button Panel Mapping
   * -------------------
   * |    ^    | 1 | 2 |
   * |   < >   | 3 | 4 |
   * |    v    | 5 | 6 |
   * |  9 | 10 | 7 | 8 |
   * | 11 | 12 |   |   |
   * -------------------
   */

  // All Constants are subject to change once we figure out what each one does in the old code. 
  public static final int TURN_TURRET_LEFT = 1;
  public static final int TURN_TURRET_RIGHT = 2;

  public static final int INTAKE_EXTEND = 3;
  public static final int INTAKE_RETRACT = 4;

  public static final int INDEX_UP = 5;
  public static final int INDEX_OUT = 6;

  public static final int INTAKE_IN = 7;
  public static final int INTAKE_OUT = 8;
  public static final int SHOOT_BUTTON = 9;
  public static final int SHOOTFALLBACK = 10;

  public static final int FLYWHEEL_ON = 11;
  public static final int SHOOT_WRONG_BALL = 12;

  public static final int TARGET_SHOOTER = 12;
    
}