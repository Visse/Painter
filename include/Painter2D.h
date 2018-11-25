#pragma once

#include <type_traits>
#include <cstdint>
#include <cmath>

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
    
    struct Vec2 {
        Vec2() = default;
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
    
    
    struct Vertex {
        Vec2 position;
        Color color;
    };
    using IndexType = uint16_t;
    
    enum class JointType : uint8_t {
        Bevel,
        Miter,
        Round
    };
    enum class CapType : uint8_t {
        Butt,
        Round,
        Square
    };
    
    struct PolyLineControllPoint {
        Vec2 pos;
        Color color;
        float weight = 1.f;
    };
    struct PolylineOptions {
        JointType joint = JointType::Miter;
        CapType startCap = CapType::Butt,
                endCap   = CapType::Butt;
    };
    
    struct PolyloopOptions {
        JointType joint = JointType::Miter;
    };
        
    
    struct Mesh {
        const Vertex *vertexes = nullptr;
        size_t vertexCount = 0;
        
        const IndexType *indexes = nullptr;
        size_t indexCount = 0;
    };
    
    class Painter2D {
    public:
        Painter2D();
        ~Painter2D();
        
        /// @todo implement copy & move
        Painter2D( const Painter2D& ) = delete;
        Painter2D( Painter2D&& ) = delete;
        Painter2D& operator = ( const Painter2D& ) = delete;
        Painter2D& operator = ( Painter2D&& ) = delete;
        
    public:
        Mesh getMesh() const;
        
        void drawQuad( Vec2 origin, Vec2 v1, Vec2 v2, Color color );
        
        void drawConvexPolygon( const Vertex *vertexes, unsigned count );
        void drawConvexPolygon( const Vec2 *positions, unsigned count, Color color );
        
        void drawPolyLine( const PolyLineControllPoint *controllPoints, unsigned count,  PolylineOptions options = PolylineOptions());
        void drawPolyLine( const Vec2 *positions, unsigned count, Color color, float thickness, PolylineOptions options = PolylineOptions() );
        
        void drawPolyLoop( const PolyLineControllPoint *controllPoints, unsigned count, PolyloopOptions options );
        void drawPolyLoop( const Vec2 *positions, unsigned count, Color color, float thickness, PolyloopOptions options );
        
    private:
        struct Impl;
        Impl *impl() {
            return reinterpret_cast<Impl*>(&mImpl);
        }
        const Impl* impl() const {
            return reinterpret_cast<const Impl*>(&mImpl);
        }
        
        static const int IMPL_SIZE = 64;
        std::aligned_storage<IMPL_SIZE, alignof(void*)>::type mImpl;
    };
}
