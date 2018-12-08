#pragma once

#include <cstdint>

namespace Painter
{
    struct Color {
        Color() = default;
        Color( uint8_t r_, uint8_t g_, uint8_t b_, uint8_t a_=255 ) :
            r(r_), g(g_), b(b_), a(a_)
        {}
        uint8_t r=0, g=0, b=0, a=255;
    };
    
    
    // 50% of each color
    inline Color mix( Color c1, Color c2 ) {
        int r = int(c1.r) + int(c2.r),
            g = int(c1.g) + int(c2.g),
            b = int(c1.b) + int(c2.b),
            a = int(c1.a) + int(c2.a);
            
        return Color {
            uint8_t(r/2), 
            uint8_t(g/2),
            uint8_t(b/2), 
            uint8_t(a/2)
        };
    }
}