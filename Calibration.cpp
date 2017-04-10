#include "Marlin.h"
#include "Calibration.h"
#include "stepper.h"
#include "language.h"

#ifdef ENABLE_AUTO_BED_LEVELING

#include "vector_3.h"
#ifdef ACCURATE_BED_LEVELING
#include "qr_solve.h"
#endif

#ifdef NONLINEAR_BED_LEVELING
float bed_level[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
#endif


static void do_blocking_move_to(float x, float y, float z);
static void print_bed_level();

static void retract_z_probe() {
    // Retract Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
        #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
        #endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);
        #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
        #endif
    }
    #else // Push up the Z probe by moving the end effector, no servo needed.
    feedrate = homing_feedrate[X_AXIS];
    destination[Z_AXIS] = current_position[Z_AXIS] + 20;
    prepare_move_raw();

    destination[X_AXIS] = 0;//-78;
    destination[Y_AXIS] = 0;//29;
    destination[Z_AXIS] = 5;//34;
    prepare_move_raw();

    // TODO: Move the nozzle down until the Z probe switch is activated.
    //enable_endstops(true);
    //destination[Z_AXIS] = current_position[Z_AXIS] - 30;
    //enable_endstops(false); 自动调平

    // Move the nozzle down further to push the probe into retracted position.
    feedrate = homing_feedrate[Z_AXIS]/10;
    destination[Z_AXIS] = current_position[Z_AXIS] - 0;
    prepare_move_raw();

    feedrate = homing_feedrate[Z_AXIS];
    destination[Z_AXIS] = current_position[Z_AXIS] + 0;
    prepare_move_raw();
    st_synchronize();
    #endif //SERVO_ENDSTOPS
}

/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before) {
    // move to right place
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
    do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

    #ifdef SERVO_ENDSTOPS
    engage_z_probe();   // Engage Z Servo endstop if available
    #endif //SERVO_ENDSTOPS

    run_z_probe();
    float measured_z = current_position[Z_AXIS];

    #ifdef SERVO_ENDSTOPS
    retract_z_probe();
    #endif //SERVO_ENDSTOPS

    SERIAL_PROTOCOLPGM(MSG_BED);
    SERIAL_PROTOCOLPGM(" x: ");
    SERIAL_PROTOCOL(x);
    SERIAL_PROTOCOLPGM(" y: ");
    SERIAL_PROTOCOL(y);
    SERIAL_PROTOCOLPGM(" z: ");
    SERIAL_PROTOCOL(measured_z);
    SERIAL_PROTOCOLPGM("\n");
    return measured_z;
}

#ifdef ACCURATE_BED_LEVELING
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    vector_3 corrected_position = plan_get_position();
//    corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#else
static void set_bed_level_equation(float z_at_xLeft_yFront, float z_at_xRight_yFront, float z_at_xLeft_yBack) {
    plan_bed_level_matrix.set_to_identity();

    vector_3 xLeftyFront = vector_3(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, z_at_xLeft_yFront);
    vector_3 xLeftyBack = vector_3(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, z_at_xLeft_yBack);
    vector_3 xRightyFront = vector_3(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, z_at_xRight_yFront);

    vector_3 xPositive = (xRightyFront - xLeftyFront).get_normal();
    vector_3 yPositive = (xLeftyBack - xLeftyFront).get_normal();
    vector_3 planeNormal = vector_3::cross(xPositive, yPositive).get_normal();

    //planeNormal.debug("planeNormal");
    //yPositive.debug("yPositive");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    // and set our bed level equation to do the right thing
    //plan_bed_level_matrix.debug("bed level after");

    vector_3 corrected_position = plan_get_position();
    //corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#endif // ACCURATE_BED_LEVELING

#ifdef NONLINEAR_BED_LEVELING
static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
  if (bed_level[x][y] != 0.0) {
    return;  // Don't overwrite good values.
  }
  float a = 2*bed_level[x+xdir][y] - bed_level[x+xdir*2][y];  // Left to right.
  float b = 2*bed_level[x][y+ydir] - bed_level[x][y+ydir*2];  // Front to back.
  float c = 2*bed_level[x+xdir][y+ydir] - bed_level[x+xdir*2][y+ydir*2];  // Diagonal.
  float median = c;  // Median is robust (ignores outliers).
  if (a < b) {
    if (b < c) median = b;
    if (c < a) median = a;
  } else {  // b <= a
    if (c < b) median = b;
    if (a < c) median = a;
  }
  bed_level[x][y] = median;
}

// Fill in the unprobed points (corners of circular print surface)
// using linear extrapolation, away from the center.
static void extrapolate_unprobed_bed_level() {
  int half = (ACCURATE_BED_LEVELING_POINTS-1)/2;
  for (int y = 0; y <= half; y++) {
    for (int x = 0; x <= half; x++) {
      if (x + y < 3) continue;
      extrapolate_one_point(half-x, half-y, x>1?+1:0, y>1?+1:0);
      extrapolate_one_point(half+x, half-y, x>1?-1:0, y>1?+1:0);
      extrapolate_one_point(half-x, half+y, x>1?+1:0, y>1?-1:0);
      extrapolate_one_point(half+x, half+y, x>1?-1:0, y>1?-1:0);
    }
  }
}
#endif //NONLINEAR_BED_LEVELING

void init_calibration() {
    #if Z_MIN_PIN == -1
    #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
    #endif

    st_synchronize();
    // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
    //vector_3 corrected_position = plan_get_position_mm();
    //corrected_position.debug("position before G29");
    plan_bed_level_matrix.set_to_identity();

    #ifdef NONLINEAR_BED_LEVELING
    reset_bed_level();
    #else
    vector_3 uncorrected_position = plan_get_position();
    //uncorrected_position.debug("position durring G29");
    current_position[X_AXIS] = uncorrected_position.x;
    current_position[Y_AXIS] = uncorrected_position.y;
    current_position[Z_AXIS] = uncorrected_position.z;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    #endif //NONLINEAR_BED_LEVELING

    #ifndef SERVO_ENDSTOPS
    engage_z_probe();   // Engage Z probe by moving the end effector.
    #endif

    feedrate = homing_feedrate[Z_AXIS];
}

void finish_calibration() {
    st_synchronize();

    #ifndef SERVO_ENDSTOPS
    retract_z_probe();   // Retract Z probe by moving the end effector.
    #endif //SERVO_ENDSTOPS

    #ifndef NONLINEAR_BED_LEVELING
    // The following code correct the Z height difference from z-probe position and hotend tip position.
    // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
    // When the bed is uneven, this height must be corrected.
    real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
    x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
    y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
    z_tmp = current_position[Z_AXIS];

    apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
    current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    #endif //NONLINEAR_BED_LEVELING
}

#ifdef ACCURATE_BED_LEVELING
void accurate_bed_leveling() {
    // solve the plane equation ax + by + d = z
    // A is the matrix with rows [x y 1] for all the probed points
    // B is the vector of the Z positions
    // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
    // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

    // "A" matrix of the linear system of equations
    double eqnAMatrix[ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS*3];
    // "B" vector of Z points
    double eqnBVector[ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS];

    #ifdef NONLINEAR_BED_LEVELING
    float z_offset = Z_PROBE_OFFSET_FROM_EXTRUDER;
    if (code_seen(axis_codes[Z_AXIS])) {
        z_offset += code_value();
    }
    #endif //NONLINEAR_BED_LEVELING

    int probePointCounter = 0;
    for (int yCount=0; yCount < ACCURATE_BED_LEVELING_POINTS; yCount++)
    {
        float yProbe = FRONT_PROBE_BED_POSITION + ACCURATE_BED_LEVELING_GRID_Y * yCount;
        int xStart, xStop, xInc;
        if (yCount % 2) {
            xStart = 0;
            xStop = ACCURATE_BED_LEVELING_POINTS;
            xInc = 1;
        } else {
            xStart = ACCURATE_BED_LEVELING_POINTS - 1;
            xStop = -1;
            xInc = -1;
        }

        for (int xCount=xStart; xCount != xStop; xCount += xInc)
        {
            float xProbe = LEFT_PROBE_BED_POSITION + ACCURATE_BED_LEVELING_GRID_X * xCount;

            #ifdef DELTA
            // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
            float distance_from_center = sqrt(xProbe*xProbe + yProbe*yProbe);
            if (distance_from_center > DELTA_PRINTABLE_RADIUS) continue;
            #endif //DELTA

            float z_before = probePointCounter == 0 ? Z_RAISE_BEFORE_PROBING :
                current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
            float measured_z = probe_pt(xProbe, yProbe, z_before);

            #ifdef NONLINEAR_BED_LEVELING
            bed_level[xCount][yCount] = measured_z + z_offset;
            #endif //NONLINEAR_BED_LEVELING

            eqnBVector[probePointCounter] = measured_z;

            eqnAMatrix[probePointCounter + 0*ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS] = xProbe;
            eqnAMatrix[probePointCounter + 1*ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS] = yProbe;
            eqnAMatrix[probePointCounter + 2*ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS] = 1;
            probePointCounter++;
        }
    }
    clean_up_after_endstop_move();

    #ifdef NONLINEAR_BED_LEVELING
    extrapolate_unprobed_bed_level();
    print_bed_level();
    #else
    // solve lsq problem
    double *plane_equation_coefficients = qr_solve(ACCURATE_BED_LEVELING_POINTS*ACCURATE_BED_LEVELING_POINTS, 3, eqnAMatrix, eqnBVector);

    SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
    SERIAL_PROTOCOL(plane_equation_coefficients[0]);
    SERIAL_PROTOCOLPGM(" b: ");
    SERIAL_PROTOCOL(plane_equation_coefficients[1]);
    SERIAL_PROTOCOLPGM(" d: ");
    SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);


    set_bed_level_equation_lsq(plane_equation_coefficients);

    free(plane_equation_coefficients);
    #endif //NONLINEAR_BED_LEVELING
}
#else
void simple_bed_leveling() {
    // prob 1
    float z_at_xLeft_yBack = probe_pt(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, Z_RAISE_BEFORE_PROBING);

    // prob 2
    float z_at_xLeft_yFront = probe_pt(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

    // prob 3
    float z_at_xRight_yFront = probe_pt(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

    clean_up_after_endstop_move();

    set_bed_level_equation(z_at_xLeft_yFront, z_at_xRight_yFront, z_at_xLeft_yBack);
}
#endif

void run_z_probe() {
    plan_bed_level_matrix.set_to_identity();

#ifdef DELTA
    enable_endstops(true);
    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    feedrate = homing_feedrate[Z_AXIS]/4;
    destination[Z_AXIS] = -10;
    prepare_move_raw();
    st_synchronize();
    endstops_hit_on_purpose();

    enable_endstops(false);
    long stop_steps = st_get_position(Z_AXIS);

    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
#else
    feedrate = homing_feedrate[Z_AXIS];

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_retract_mm(Z_AXIS);
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    // move back down slowly to find bed
    feedrate = homing_feedrate[Z_AXIS]/4;
    zPosition -= home_retract_mm(Z_AXIS) * 2;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#endif
}

void engage_z_probe() {
    // Engage Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
        #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
        #endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
        #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
        #endif
    }
    #else // Deploy the Z probe by touching the belt, no servo needed.
    feedrate = homing_feedrate[X_AXIS];
    destination[X_AXIS] = 35;
    destination[Y_AXIS] = 35;
    destination[Z_AXIS] = 100;
    prepare_move_raw();

    feedrate = homing_feedrate[X_AXIS]/10;
    destination[X_AXIS] = 0;
    prepare_move_raw();
    st_synchronize();
    #endif //SERVO_ENDSTOPS
}



// Print calibration results for plotting or manual frame adjustment.
static void print_bed_level() {
    for (int y = 0; y < ACCURATE_BED_LEVELING_POINTS; y++) {
        for (int x = 0; x < ACCURATE_BED_LEVELING_POINTS; x++) {
            SERIAL_PROTOCOL_F(bed_level[x][y], 2);
            SERIAL_PROTOCOLPGM(" ");
        }
        SERIAL_ECHOLN("");
    }
}


// Reset calibration results to zero.
void reset_bed_level() {
    //Reset the plane ("erase" all leveling data)  
    plan_bed_level_matrix.set_to_identity();  

    #ifdef NONLINEAR_BED_LEVELING  
    for (int y = 0; y < ACCURATE_BED_LEVELING_POINTS; y++) {
        for (int x = 0; x < ACCURATE_BED_LEVELING_POINTS; x++) {
            bed_level[x][y] = 0.0;
        }
    }
    #endif
}


static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = XY_TRAVEL_SPEED;

#ifdef DELTA
    destination[X_AXIS] = x;
    destination[Y_AXIS] = y;
    destination[Z_AXIS] = z;
    prepare_move_raw();
#else  // cartesian
    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    current_position[Z_AXIS] = z;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
#endif //DELTA
    st_synchronize();

    feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
    do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    refresh_cmd_timeout();

    #ifndef DELTA
        enable_endstops(true);
    #endif //Delta printers enable endstops only during Z probe down move.
}

void clean_up_after_endstop_move() {
    #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
    #endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    refresh_cmd_timeout();
}
#endif // ENABLE_AUTO_BED_LEVELING


