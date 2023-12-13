#ifndef DISPLAY_H_
#define DISPLAY_H_

#define NUM_GRID_LINES 7
#define GRID_GAP_X ((SCREEN_WIDTH - 1) / (float)NUM_GRID_LINES)
#define GRID_GAP_Y ((SCREEN_HEIGHT - 1) / (float)NUM_GRID_LINES)

void display_init();
void display_task_func();

#endif

