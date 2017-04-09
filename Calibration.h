#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifdef NONLINEAR_BED_LEVELING
float bed_level[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
#endif

static void init_calibration();
static void finish_calibration();
static void accurate_bed_leveling();
static void simple_bed_leveling();
static void run_z_probe();
static void engage_z_probe();
static void reset_bed_level();
static void retract_z_probe();
static void setup_for_endstop_move();
static void clean_up_after_endstop_move();

#endif