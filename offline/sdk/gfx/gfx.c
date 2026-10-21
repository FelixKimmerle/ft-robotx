//=============================================================================
// Graphics library for the ROBO TX Controller's display; see robotx_gfx.h.
//
// The firmware shows a frame of 128 x 64 pixels, stored row by row: 16 bytes per row, top row first, the highest
// bit of each byte is the leftmost pixel, a set bit is a dark pixel (determined with test patterns on a
// controller with firmware 1.30).
//=============================================================================

#include "robotx_gfx.h"

#define BYTES_PER_ROW (GFX_WIDTH / 8)

static UCHAR8 drawing[MAX_FRAME_SIZE]; // what the program draws into
static UCHAR8 shown[MAX_FRAME_SIZE];   // what the firmware displays

extern const UCHAR8 gfx_font5x7[95][5];

void gfx_begin(TA * p_ta)
{
    gfx_clear();
    p_ta->display.display_frame.frame = shown;
    p_ta->display.display_frame.is_pgm_master_of_display += 1;
    gfx_show(p_ta);
}

void gfx_end(TA * p_ta)
{
    p_ta->display.display_frame.is_pgm_master_of_display -= 1;
}

void gfx_show(TA * p_ta)
{
    for (int i = 0; i < MAX_FRAME_SIZE; ++i)
    {
        shown[i] = drawing[i];
    }
    p_ta->display.display_frame.id += 1; // tells the firmware there is a new frame
}

void gfx_clear(void)
{
    for (int i = 0; i < MAX_FRAME_SIZE; ++i)
    {
        drawing[i] = 0;
    }
}

void gfx_pixel(int x, int y, int color)
{
    if (x < 0 || x >= GFX_WIDTH || y < 0 || y >= GFX_HEIGHT)
    {
        return;
    }
    UCHAR8 * byte = &drawing[y * BYTES_PER_ROW + x / 8];
    UCHAR8 mask = (UCHAR8)(0x80 >> (x % 8));
    if (color == GFX_BLACK)
    {
        *byte |= mask;
    }
    else if (color == GFX_WHITE)
    {
        *byte &= (UCHAR8)~mask;
    }
    else
    {
        *byte ^= mask;
    }
}

int gfx_get_pixel(int x, int y)
{
    if (x < 0 || x >= GFX_WIDTH || y < 0 || y >= GFX_HEIGHT)
    {
        return GFX_WHITE;
    }
    return (drawing[y * BYTES_PER_ROW + x / 8] & (0x80 >> (x % 8))) ? GFX_BLACK : GFX_WHITE;
}

void gfx_line(int x0, int y0, int x1, int y1, int color)
{
    // Bresenham
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy = y1 > y0 ? y0 - y1 : y1 - y0;
    int step_x = x0 < x1 ? 1 : -1;
    int step_y = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (1)
    {
        gfx_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1)
        {
            return;
        }
        int e2 = 2 * error;
        if (e2 >= dy)
        {
            error += dy;
            x0 += step_x;
        }
        if (e2 <= dx)
        {
            error += dx;
            y0 += step_y;
        }
    }
}

void gfx_rect(int x, int y, int width, int height, int color)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }
    for (int i = 0; i < width; ++i)
    {
        gfx_pixel(x + i, y, color);
        if (height > 1)
        {
            gfx_pixel(x + i, y + height - 1, color);
        }
    }
    for (int i = 1; i < height - 1; ++i)
    {
        gfx_pixel(x, y + i, color);
        if (width > 1)
        {
            gfx_pixel(x + width - 1, y + i, color);
        }
    }
}

void gfx_fill_rect(int x, int y, int width, int height, int color)
{
    for (int row = y; row < y + height; ++row)
    {
        for (int column = x; column < x + width; ++column)
        {
            gfx_pixel(column, row, color);
        }
    }
}

void gfx_circle(int center_x, int center_y, int radius, int color)
{
    // Midpoint circle; each of the eight symmetric points is drawn once
    int x = radius;
    int y = 0;
    int error = 1 - radius;
    while (x >= y)
    {
        int points[8][2] = {{x, y}, {y, x}, {-y, x}, {-x, y}, {-x, -y}, {-y, -x}, {y, -x}, {x, -y}};
        for (int i = 0; i < 8; ++i)
        {
            // skip duplicates on the axes and diagonals, so GFX_INVERT doesn't cancel itself out
            int duplicate = 0;
            for (int j = 0; j < i; ++j)
            {
                duplicate |= points[j][0] == points[i][0] && points[j][1] == points[i][1];
            }
            if (!duplicate)
            {
                gfx_pixel(center_x + points[i][0], center_y + points[i][1], color);
            }
        }
        ++y;
        if (error < 0)
        {
            error += 2 * y + 1;
        }
        else
        {
            --x;
            error += 2 * (y - x) + 1;
        }
    }
}

void gfx_fill_circle(int center_x, int center_y, int radius, int color)
{
    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            if (dx * dx + dy * dy <= radius * radius + radius)
            {
                gfx_pixel(center_x + dx, center_y + dy, color);
            }
        }
    }
}

static void draw_char(int x, int y, char c, int scale, int color)
{
    if (c < ' ' || c > '~')
    {
        c = '?';
    }
    const UCHAR8 * glyph = gfx_font5x7[c - ' '];
    for (int column = 0; column < 5; ++column)
    {
        for (int row = 0; row < 7; ++row)
        {
            if (glyph[column] & (1 << row))
            {
                gfx_fill_rect(x + column * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

int gfx_text(int x, int y, const char * text, int scale, int color)
{
    if (scale < 1)
    {
        scale = 1;
    }
    int start_x = x;
    for (; *text; ++text)
    {
        if (*text == '\n')
        {
            x = start_x;
            y += GFX_CHAR_HEIGHT * scale;
            continue;
        }
        draw_char(x, y, *text, scale, color);
        x += GFX_CHAR_WIDTH * scale;
    }
    return x;
}

int gfx_number(int x, int y, long value, int scale, int color)
{
    char text[12];
    char * p = text + sizeof(text) - 1;
    *p = '\0';
    unsigned long magnitude = value < 0 ? 0UL - (unsigned long)value : (unsigned long)value;
    do
    {
        *--p = (char)('0' + magnitude % 10);
        magnitude /= 10;
    } while (magnitude != 0);
    if (value < 0)
    {
        *--p = '-';
    }
    return gfx_text(x, y, p, scale, color);
}

int gfx_text_width(const char * text, int scale)
{
    int longest = 0;
    int current = 0;
    for (; *text; ++text)
    {
        current = *text == '\n' ? 0 : current + 1;
        longest = current > longest ? current : longest;
    }
    return longest * GFX_CHAR_WIDTH * (scale < 1 ? 1 : scale);
}
