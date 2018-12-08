#pragma once


#include <cmath>

#if PAINTER_USE_GLM
#include <glm/vec2.hpp>
#endif

namespace Painter
{
#ifndef PAINTER_USE_GLM
    struct Vec2 {
        Vec2() = default;
        Vec2( const Vec2& ) = default;
        Vec2& operator = ( const Vec2& ) = default;
        
        Vec2( float x_, float y_ ) :
            x(x_), y(y_)
        {}
        
        float x=0.f, y=0.f;
    };
    
    inline Vec2 operator - ( const Vec2 &lhs, const Vec2 &rhs ) {
        return Vec2{
            lhs.x - rhs.x,
            lhs.y - rhs.y
        };
    }
    inline Vec2 operator + ( const Vec2 &lhs, const Vec2 &rhs ) {
        return Vec2{
            lhs.x + rhs.x,
            lhs.y + rhs.y
        };
    }
    
    inline Vec2 operator * ( const Vec2 &lhs, float rhs ) {
        return Vec2{
            lhs.x * rhs,
            lhs.y * rhs
        };
    }
#else
    typedef glm::vec2 Vec2;
#endif
    inline bool isZero( Vec2 v ) {
        return std::abs(v.x) < 1e-3f &&
               std::abs(v.y) < 1e-3f;
    }
    
    inline Vec2 perp( Vec2 v ) {
        return Vec2{v.y, -v.x};
    }
    
    inline Vec2 normalize( Vec2 v ) {
        float x = v.x,
              y = v.y;
        
        float l = std::sqrt(x*x + y*y);
        return Vec2{x/l, y/l};
    }
    
    inline float dot( Vec2 v1, Vec2 v2 ) {
        return v1.x * v2.x + v1.y * v2.y;
    }
    
    inline float cross( Vec2 v1, Vec2 v2 ) {
        return v1.x * v2.y - v1.y * v2.x;
    }
}