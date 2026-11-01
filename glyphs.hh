#pragma once

enum GlyphId {
    G_INFO,
    G_WARN,
    G_FOLD_OPEN,
    G_FOLD_CLOSED,
    G_CHECK,
    G_CROSS,
    G_ARROW_UP,
    G_ARROW_DOWN,
    G_ARROW_LEFT,
    G_ARROW_RIGHT,
    G_BULLET,
    G_SELECTED,
    G_UNSELECTED,
    G_SEPARATOR,
    G_COUNT
};

struct Glyph {
    const char* name;
    const char* text;
    const char* unicode;
};

static const Glyph GLYPHS[G_COUNT] = {
    {"info",        "(i)", "\u24D8"},
    {"warning",     "(!)", "\u26A0\uFE0E"},
    {"fold_open",   "[-]", "\u25BE"},
    {"fold_closed", "[+]", "\u25B8"},
    {"check",       "=",   "\u2713"},
    {"cross",       "X",   "\u2717"},
    {"arrow_up",    "^",   "\u2191"},
    {"arrow_down",  "v",   "\u2193"},
    {"arrow_left",  "<",   "\u2190"},
    {"arrow_right", ">",   "\u2192"},
    {"bullet",      "*",   "\u2022"},
    {"selected",    "[*]", "\u25CF"},
    {"unselected",  "[ ]", "\u25CB"},
    {"separator",   "|",   "\u00B7"},
};

inline const char* glyph(GlyphId id, bool unicode) {
    return unicode ? GLYPHS[id].unicode : GLYPHS[id].text;
}
