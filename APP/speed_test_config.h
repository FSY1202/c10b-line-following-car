#ifndef __SPEED_TEST_CONFIG_H
#define __SPEED_TEST_CONFIG_H

// Change only this value when testing 240, 270, or 300 mm/s.
#define SPEED_TEST_TARGET_MM_S       (300)

// 1 mm/s every 10 ms = 100 mm/s^2. The car reaches 300 mm/s in about 3 s.
#define SPEED_TEST_RAMP_PERIOD_MS    (10u)
#define SPEED_TEST_RAMP_STEP_MM_S    (1)

#endif
