//=============================================================================
// Small graphics library for the ROBO TX Controller's display (128 x 64 pixels, monochrome), for programs in
// download (offline) mode.
//
//   gfx_begin(p_ta);                 // in PrgInit(): take over the display
//   gfx_clear();
//   gfx_text(0, 0, "Hello", 1, GFX_BLACK);
//   gfx_rect(0, 10, 50, 20, GFX_BLACK);
//   gfx_show(p_ta);                  // show what was drawn
//
// Drawing happens in a buffer of its own; gfx_show() hands a finished copy to the firmware, so it never shows a
// half-drawn picture. Coordinates outside the display are clipped. Don't call gfx_show() every millisecond;
// a few times per second is plenty for a display.
//=============================================================================

#ifndef ROBOTX_GFX_H
#define ROBOTX_GFX_H

#include "ROBO_TX_PRG.h"

#define GFX_WIDTH       128
#define GFX_HEIGHT      64

// Colors
#define GFX_WHITE       0   // pixel off
#define GFX_BLACK       1   // pixel on
#define GFX_INVERT      2   // flip the pixel

// Size of a character at scale 1: 5 x 7 pixels in a cell of 6 x 8 pixels.
#define GFX_CHAR_WIDTH  6
#define GFX_CHAR_HEIGHT 8

// Takes over the display: from now on it shows the pictures from gfx_show() instead of the firmware's menus.
void gfx_begin(TA * p_ta);

// Gives the display back to the firmware.
void gfx_end(TA * p_ta);

// Shows everything drawn so far.
void gfx_show(TA * p_ta);

// Clears the drawing buffer (all pixels white).
void gfx_clear(void);

void gfx_pixel(int x, int y, int color);
int gfx_get_pixel(int x, int y); // GFX_WHITE or GFX_BLACK; GFX_WHITE outside the display

void gfx_line(int x0, int y0, int x1, int y1, int color);
void gfx_rect(int x, int y, int width, int height, int color);      // outline
void gfx_fill_rect(int x, int y, int width, int height, int color); // filled
void gfx_circle(int center_x, int center_y, int radius, int color);
void gfx_fill_circle(int center_x, int center_y, int radius, int color);

// Draws text (printable ASCII) with its top-left corner at x, y; scale 1 = 6 x 8 pixels per character, 2 = 12 x 16,
// ... Only the characters' pixels are drawn; clear the background with gfx_fill_rect() first if needed. '\n' starts
// a new line. Returns the x position after the last character.
int gfx_text(int x, int y, const char * text, int scale, int color);

// Draws a whole number, e.g. a counter or a sensor value. Returns the x position after the last digit.
int gfx_number(int x, int y, long value, int scale, int color);

// Width in pixels of text at the given scale (longest line).
int gfx_text_width(const char * text, int scale);

#endif // ROBOTX_GFX_H
