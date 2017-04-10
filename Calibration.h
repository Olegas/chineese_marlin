#ifndef CALIBRATION_H
#define CALIBRATION_H

#ifdef NONLINEAR_BED_LEVELING
extern float bed_level[ACCURATE_BED_LEVELING_POINTS][ACCURATE_BED_LEVELING_POINTS];
#endif

void init_calibration();
void finish_calibration();
void accurate_bed_leveling();
void simple_bed_leveling();
void run_z_probe();
void engage_z_probe();
void reset_bed_level();
void retract_z_probe();
void setup_for_endstop_move();
void clean_up_after_endstop_move();

#endif