#pragma once

#define CONFIGURATION_H_VERSION 020005

// @section info

#define STRING_CONFIG_H_AUTHOR "Stanislav Petr"
#define SHOW_BOOTSCREEN

// @section machine

#define SERIAL_PORT 0
#define BAUDRATE 250000
#define MOTHERBOARD BOARD_TRIGORILLA_14_11
#define CUSTOM_MACHINE_NAME "DigitalData D1"
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#define EXTRUDERS 1
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75

// @section temperature

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_BED 1
#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100
//#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

#define TEMP_RESIDENCY_TIME     10
#define TEMP_WINDOW              1
#define TEMP_HYSTERESIS          3
#define TEMP_BED_RESIDENCY_TIME 10
#define TEMP_BED_WINDOW          1
#define TEMP_BED_HYSTERESIS      3

#define HEATER_0_MINTEMP   5
#define HEATER_1_MINTEMP   5
#define HEATER_2_MINTEMP   5
#define HEATER_3_MINTEMP   5
#define HEATER_4_MINTEMP   5
#define HEATER_5_MINTEMP   5
#define HEATER_6_MINTEMP   5
#define HEATER_7_MINTEMP   5
#define BED_MINTEMP        5

#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP      120

//===========================================================================
//============================= PID Settings ================================
//===========================================================================

#define PIDTEMP
#define BANG_MAX 255
#define PID_MAX BANG_MAX
#define PID_K1 0.95
#define PID_EDIT_MENU
#define PID_AUTOTUNE_MENU
#define PID_FUNCTIONAL_RANGE 15
// Anycubic Kossel - run 'M106 S255' & 'M303 E0 C10 S200'
#define DEFAULT_Kp 19.69
#define DEFAULT_Ki 1.53
#define DEFAULT_Kd 63.30

#define PIDTEMPBED
#define MAX_BED_POWER 255
// Anycubic Kossel - run 'M303 E-1 C8 S60'
#define DEFAULT_bedKp 194.80
#define DEFAULT_bedKi 37.75
#define DEFAULT_bedKd 670.26

// @section extruder

#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 750
#define THERMAL_PROTECTION_HOTENDS
#define THERMAL_PROTECTION_BED

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================

#define DELTA
#define DELTA_SEGMENTS_PER_SECOND 100
#define DELTA_CALIBRATION_MENU
#define DELTA_AUTO_CALIBRATION
#define DELTA_CALIBRATION_DEFAULT_POINTS 7
#define PROBE_MANUALLY_STEP 0.025      // (mm)
#define DELTA_PRINTABLE_RADIUS 110.0  // (mm)
#define DELTA_DIAGONAL_ROD 267        // (mm)
#define DELTA_RADIUS 133.22
#define DELTA_HEIGHT 320.00             // (mm) Get this value from G33 auto calibrate
#define DELTA_ENDSTOP_ADJ { 0.0, 0.0, 0.0 } // Get these values from G33 auto calibrate
#define DELTA_TOWER_ANGLE_TRIM { 0.0, 0.0, 0.0 } // Get these values from G33 auto calibrate
#define DELTA_RADIUS_TRIM_TOWER { 0.0, 0.0, 0.0 }
#define DELTA_DIAGONAL_ROD_TRIM_TOWER { 0.0, 0.0, 0.0 }

//===========================================================================
//============================== Endstop Settings ===========================
//===========================================================================

// @section homing

#define USE_ZMIN_PLUG
#define USE_XMAX_PLUG
#define USE_YMAX_PLUG
#define USE_ZMAX_PLUG

#define ENDSTOPPULLUP_XMAX
#define ENDSTOPPULLUP_YMAX
#define ENDSTOPPULLUP_ZMAX
#define ENDSTOP_INTERRUPTS_FEATURE

#define X_MAX_ENDSTOP_INVERTING false
#define Y_MAX_ENDSTOP_INVERTING false
#define Z_MAX_ENDSTOP_INVERTING false
#define Z_MIN_ENDSTOP_INVERTING false
#define Z_MIN_PROBE_ENDSTOP_INVERTING Z_MIN_ENDSTOP_INVERTING

/**
 * Stepper Drivers
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced options for
 * stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 *
 * A4988 is assumed for unspecified drivers.
 *
 * Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
 *          TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'L6470', 'L6474', 'POWERSTEP01', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160', 'TMC5160_STANDALONE']
 */
#define X_DRIVER_TYPE  TMC2209_STANDALONE
#define Y_DRIVER_TYPE  TMC2209_STANDALONE
#define Z_DRIVER_TYPE  TMC2209_STANDALONE
#define E0_DRIVER_TYPE TMC2209_STANDALONE

//=============================================================================
//============================== Movement Settings ============================
//=============================================================================
// @section motion

#define XYZ_FULL_STEPS_PER_ROTATION 200
#define XYZ_MICROSTEPS 16
#define XYZ_BELT_PITCH 2
#define XYZ_PULLEY_TEETH 20
#define DEFAULT_XYZ_STEPS_PER_UNIT ((XYZ_FULL_STEPS_PER_ROTATION) * (XYZ_MICROSTEPS) / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH)) // 80
#define DEFAULT_AXIS_STEPS_PER_UNIT   { DEFAULT_XYZ_STEPS_PER_UNIT, DEFAULT_XYZ_STEPS_PER_UNIT, DEFAULT_XYZ_STEPS_PER_UNIT, 96 }  // default steps per unit for Kossel (GT2, 20 tooth)
#define DEFAULT_MAX_FEEDRATE          { 1000, 1000, 1000, 200}
#define DEFAULT_MAX_ACCELERATION       { 2000, 2000, 2000, 2000 }
#define LIMITED_MAX_ACCEL_EDITING
#define MAX_ACCEL_EDIT_VALUES       { 6000, 6000, 6000, 6000 }
#define DEFAULT_ACCELERATION          600
#define DEFAULT_RETRACT_ACCELERATION  3000
#define DEFAULT_TRAVEL_ACCELERATION   1000
#define CLASSIC_JERK
#define DEFAULT_XJERK  5.0
#define DEFAULT_YJERK  DEFAULT_XJERK
#define DEFAULT_ZJERK  DEFAULT_XJERK
#define LIMITED_JERK_EDITING
#define MAX_JERK_EDIT_VALUES { 20, 20, 20, 10 }
#define DEFAULT_EJERK    5.0
#define S_CURVE_ACCELERATION

//===========================================================================
//============================= Z Probe Options =============================
//===========================================================================
// @section probes

//
// See http://marlinfw.org/docs/configuration/probes.html
//

/**
 * Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
 *
 * Enable this option for a probe connected to the Z Min endstop pin.
 */
#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN

/**
 * Z_MIN_PROBE_PIN
 *
 * Define this pin if the probe is not connected to Z_MIN_PIN.
 * If not defined the default pin for the selected MOTHERBOARD
 * will be used. Most of the time the default is what you want.
 *
 *  - The simplest option is to use a free endstop connector.
 *  - Use 5V for powered (usually inductive) sensors.
 *
 *  - RAMPS 1.3/1.4 boards may use the 5V, GND, and Aux4->D32 pin:
 *    - For simple switches connect...
 *      - normally-closed switches to GND and D32.
 *      - normally-open switches to 5V and D32.
 *
 */
//#define Z_MIN_PROBE_PIN 32 // Pin 32 is the RAMPS default

/**
 * Probe Type
 *
 * Allen Key Probes, Servo Probes, Z-Sled Probes, FIX_MOUNTED_PROBE, etc.
 * Activate one of these to use Auto Bed Leveling below.
 */


#define FIX_MOUNTED_PROBE
#define NOZZLE_TO_PROBE_OFFSET { 38, -28, 0 }
#define MIN_PROBE_EDGE 15
#define XY_PROBE_SPEED 6000
#define Z_PROBE_SPEED_FAST (HOMING_FEEDRATE_Z / 2)
#define Z_PROBE_SPEED_SLOW (HOMING_FEEDRATE_Z / 4)
#define MULTIPLE_PROBING 4
#define EXTRA_PROBING    1

/**
 * Z probes require clearance when deploying, stowing, and moving between
 * probe points to avoid hitting the bed and other hardware.
 * Servo-mounted probes require extra space for the arm to rotate.
 * Inductive probes need space to keep from triggering early.
 *
 * Use these settings to specify the distance (mm) to raise the probe (or
 * lower the bed). The values set here apply over and above any (negative)
 * probe Z Offset set with NOZZLE_TO_PROBE_OFFSET, M851, or the LCD.
 * Only integer values >= 1 are valid here.
 *
 * Example: `M851 Z-5` with a CLEARANCE of 4  =>  9mm from bed to nozzle.
 *     But: `M851 Z+1` with a CLEARANCE of 2  =>  2mm from bed to nozzle.
 */
#define Z_CLEARANCE_DEPLOY_PROBE   0 // Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES 1 // Z Clearance between probe points
#define Z_CLEARANCE_MULTI_PROBE    1 // Z Clearance between multiple probes
//#define Z_AFTER_PROBING          5 // Z position after probing is done
#define Z_PROBE_LOW_POINT          -3

// For M851 give a range for adjusting the Z probe offset
#define Z_PROBE_OFFSET_RANGE_MIN -40
#define Z_PROBE_OFFSET_RANGE_MAX 20

#define Z_MIN_PROBE_REPEATABILITY_TEST

/**
 * Enable one or more of the following if probing seems unreliable.
 * Heaters and/or fans can be disabled during probing to minimize electrical
 * noise. A delay can also be added to allow noise and vibration to settle.
 * These options are most useful for the BLTouch probe, but may also improve
 * readings with inductive probes and piezo sensors.
 */
#define PROBING_HEATERS_OFF       // Turn heaters off when probing
#if ENABLED(PROBING_HEATERS_OFF)
  //#define WAIT_FOR_BED_HEATER     // Wait for bed to heat back up between probes (to improve accuracy)
#endif
//#define PROBING_FANS_OFF          // Turn fans off when probing
//#define PROBING_STEPPERS_OFF      // Turn steppers off (unless needed to hold position) when probing
//#define DELAY_BEFORE_PROBING 200  // (ms) To prevent vibrations from triggering piezo sensors

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{ 0:'Low', 1:'High' }
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis stepper immediately when it's not being used.
// WARNING: When motors turn off there is a chance of losing position accuracy!
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false

// Warn on display about possibly reduced accuracy
//#define DISABLE_REDUCED_ACCURACY_WARNING

// @section extruder

#define DISABLE_E false             // For all extruders
#define DISABLE_INACTIVE_EXTRUDER   // Keep only the active extruder enabled

// @section machine

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define INVERT_E6_DIR false
#define INVERT_E7_DIR false

// @section homing

//#define NO_MOTION_BEFORE_HOMING // Inhibit movement until all axes have been homed

//#define UNKNOWN_Z_NO_RAISE      // Don't raise Z (lower the bed) if Z is "unknown." For beds that fall when Z is powered off.

//#define Z_HOMING_HEIGHT  4      // (mm) Minimal Z height before homing (G28) for Z clearance above the bed, clamps, ...
                                  // Be sure to have this much clearance over your Z_MAX_POS to prevent grinding.

//#define Z_AFTER_HOMING  10      // (mm) Height to move to after homing Z

// Direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#define X_HOME_DIR 1  // deltas always home to max
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// @section machine

// The size of the print bed
#define X_BED_SIZE ((DELTA_PRINTABLE_RADIUS) * 2)
#define Y_BED_SIZE ((DELTA_PRINTABLE_RADIUS) * 2)

// Travel limits (mm) after homing, corresponding to endstop positions.
#define X_MIN_POS -(DELTA_PRINTABLE_RADIUS)
#define Y_MIN_POS -(DELTA_PRINTABLE_RADIUS)
#define Z_MIN_POS 0
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS

/**
 * Software Endstops
 *
 * - Prevent moves outside the set machine bounds.
 * - Individual axes can be disabled, if desired.
 * - X and Y only apply to Cartesian robots.
 * - Use 'M211' to set software endstops on/off or report current state
 */

// Min software endstops constrain movement within minimum coordinate bounds
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
#endif

// Max software endstops constrain movement within maximum coordinate bounds
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
#endif

#if EITHER(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
  #define SOFT_ENDSTOPS_MENU_ITEM  // Enable/Disable software endstops from the LCD
#endif

//===========================================================================
//=============================== Bed Leveling ==============================
//===========================================================================
// @section calibrate

/**
 * Choose one of the options below to enable G29 Bed Leveling. The parameters
 * and behavior of G29 will change depending on your selection.
 *
 *  If using a Probe for Z Homing, enable Z_SAFE_HOMING also!
 *
 * - AUTO_BED_LEVELING_3POINT
 *   Probe 3 arbitrary points on the bed (that aren't collinear)
 *   You specify the XY coordinates of all 3 points.
 *   The result is a single tilted plane. Best for a flat bed.
 *
 * - AUTO_BED_LEVELING_LINEAR
 *   Probe several points in a grid.
 *   You specify the rectangle and the density of sample points.
 *   The result is a single tilted plane. Best for a flat bed.
 *
 * - AUTO_BED_LEVELING_BILINEAR
 *   Probe several points in a grid.
 *   You specify the rectangle and the density of sample points.
 *   The result is a mesh, best for large or uneven beds.
 *
 * - AUTO_BED_LEVELING_UBL (Unified Bed Leveling)
 *   A comprehensive bed leveling system combining the features and benefits
 *   of other systems. UBL also includes integrated Mesh Generation, Mesh
 *   Validation and Mesh Editing systems.
 *
 * - MESH_BED_LEVELING
 *   Probe a grid manually
 *   The result is a mesh, suitable for large or uneven beds. (See BILINEAR.)
 *   For machines without a probe, Mesh Bed Leveling provides a method to perform
 *   leveling in steps so you can manually adjust the Z height at each grid-point.
 *   With an LCD controller the process is guided step-by-step.
 */
//#define AUTO_BED_LEVELING_LINEAR
#define AUTO_BED_LEVELING_BILINEAR
//#define AUTO_BED_LEVELING_UBL
//#define MESH_BED_LEVELING

#define RESTORE_LEVELING_AFTER_G28

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL)
  // Gradually reduce leveling correction until a set height is reached,
  // at which point movement will be level to the machine's XY plane.
  // The height can be set with M420 Z<height>
  //#define ENABLE_LEVELING_FADE_HEIGHT

  // For Cartesian machines, instead of dividing moves on mesh boundaries,
  // split up moves into short segments like a Delta. This follows the
  // contours of the bed more closely than edge-to-edge straight moves.
  #define SEGMENT_LEVELED_MOVES
  #define LEVELED_SEGMENT_LENGTH 5.0 // (mm) Length of all segments (except the last one)

  /**
   * Enable the G26 Mesh Validation Pattern tool.
   */
  #define G26_MESH_VALIDATION
  #if ENABLED(G26_MESH_VALIDATION)
    #define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.
    #define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for the G26 Mesh Validation Tool.
    #define MESH_TEST_HOTEND_TEMP  205    // (°C) Default nozzle temperature for the G26 Mesh Validation Tool.
    #define MESH_TEST_BED_TEMP      60    // (°C) Default bed temperature for the G26 Mesh Validation Tool.
    #define G26_XY_FEEDRATE         20    // (mm/s) Feedrate for XY Moves for the G26 Mesh Validation Tool.
    #define G26_RETRACT_MULTIPLIER   1.0  // G26 Q (retraction) used by default between mesh test elements.
  #endif

#endif

#if EITHER(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)

  // Set the number of grid points per dimension.
  // Works best with 5 or more points in each dimension.
  #define GRID_MAX_POINTS_X 9
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  // Probe along the Y axis, advancing X after each column
  //#define PROBE_Y_FIRST

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    // Beyond the probed grid, continue the implied tilt?
    // Default is to maintain the height of the nearest edge.
    //#define EXTRAPOLATE_BEYOND_GRID

    //
    // Experimental Subdivision of the grid by Catmull-Rom method.
    // Synthesizes intermediate points to produce a more detailed mesh.
    //
    //#define ABL_BILINEAR_SUBDIVISION
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      // Number of subdivisions between probe points
      #define BILINEAR_SUBDIVISIONS 3
    #endif

  #endif

#elif ENABLED(AUTO_BED_LEVELING_UBL)

  //===========================================================================
  //========================= Unified Bed Leveling ============================
  //===========================================================================

  //#define MESH_EDIT_GFX_OVERLAY   // Display a graphics overlay while editing the mesh

  #define MESH_INSET 1              // Set Mesh bounds as an inset region of the bed
  #define GRID_MAX_POINTS_X 10      // Don't use more than 15 points per axis, implementation limited.
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  #define UBL_MESH_EDIT_MOVES_Z     // Sophisticated users prefer no movement of nozzle
  #define UBL_SAVE_ACTIVE_ON_M500   // Save the currently active mesh in the current slot on M500

  //#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5 // When the nozzle is off the mesh, this value is used
                                          // as the Z-Height correction value.

#elif ENABLED(MESH_BED_LEVELING)

  //===========================================================================
  //=================================== Mesh ==================================
  //===========================================================================

  #define MESH_INSET 10          // Set Mesh bounds as an inset region of the bed
  #define GRID_MAX_POINTS_X 3    // Don't use more than 7 points per axis, implementation limited.
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  //#define MESH_G28_REST_ORIGIN // After homing all axes ('G28' or 'G28 XYZ') rest Z at Z_MIN_POS

#endif // BED_LEVELING

/**
 * Add a bed leveling sub-menu for ABL or MBL.
 * Include a guided procedure if manual probing is enabled.
 */
#define LCD_BED_LEVELING

#if ENABLED(LCD_BED_LEVELING)
  #define MESH_EDIT_Z_STEP  0.05  // (mm) Step size while manually probing Z axis.
  #define LCD_PROBE_Z_RANGE 4     // (mm) Z Range centered on Z_MIN_POS for LCD Z adjustment
  //#define MESH_EDIT_MENU        // Add a menu to edit mesh points
#endif

/**
 * Commands to execute at the end of G29 probing.
 * Useful to retract or move the Z probe out of the way.
 */
//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10"


// @section homing

#define BED_CENTER_AT_0_0
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS DELTA_HEIGHT // Distance between the nozzle to printbed after homing

#define HOMING_FEEDRATE_Z  (6000)
#define VALIDATE_HOMING_ENDSTOPS


//=============================================================================
//============================= Additional Features ===========================
//=============================================================================

// @section extras

#define EEPROM_SETTINGS
#define DISABLE_M503
#define EEPROM_CHITCHAT
#define EEPROM_AUTO_INIT
#define HOST_KEEPALIVE_FEATURE
#define DEFAULT_KEEPALIVE_INTERVAL 10
#define BUSY_WHILE_HEATING

// @section temperature

#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 190
#define PREHEAT_1_TEMP_BED     60
#define PREHEAT_1_FAN_SPEED   255

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    100
#define PREHEAT_2_FAN_SPEED   255

/**
 * Nozzle Park
 *
 * Park the nozzle at the given XYZ position on idle or G27.
 *
 * The "P" parameter controls the action applied to the Z axis:
 *
 *    P0  (Default) If Z is below park Z raise the nozzle.
 *    P1  Raise the nozzle always to Z-park height.
 *    P2  Raise the nozzle by Z-park amount, limited to Z_MAX_POS.
 */
#define NOZZLE_PARK_FEATURE

#if ENABLED(NOZZLE_PARK_FEATURE)
  // Specify a park position as { X, Y, Z_raise }
  #define NOZZLE_PARK_POINT { 0, 0, 20 }
  #define NOZZLE_PARK_XY_FEEDRATE 100
  #define NOZZLE_PARK_Z_FEEDRATE 100
#endif

#define PRINTJOB_TIMER_AUTOSTART
#define PRINTCOUNTER

//=============================================================================
//============================= LCD and SD support ============================
//=============================================================================

// @section lcd

#define LCD_LANGUAGE en
#define DISPLAY_CHARSET_HD44780 JAPANESE
#define LCD_INFO_SCREEN_STYLE 1
#define SDSUPPORT
#define SD_CHECK_AND_RETRY
#define ENCODER_PULSES_PER_STEP 3
#define ENCODER_STEPS_PER_MENU_ITEM 1
#define REVERSE_ENCODER_DIRECTION
//#define REVERSE_MENU_DIRECTION
//#define REVERSE_SELECT_DIRECTION
#define SPEAKER
#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
#define LCD_FEEDBACK_FREQUENCY_HZ 5000
#define REPRAP_DISCOUNT_SMART_CONTROLLER
