#pragma once

#include <type_traits>
#include <cstdint>
#include <cmath>
#include <initializer_list>

#include "Color.h"
#include "Vec2.h"

namespace Painter
{
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
        
    struct MeshSection {
        size_t firstVertex = 0,
               vertexCount = 0,
               firstIndex  = 0,
               indexCount  = 0;
    };
    struct Mesh {
        const Vertex *vertexes = nullptr;
        size_t vertexCount = 0;
        
        const IndexType *indexes = nullptr;
        size_t indexCount = 0;
        
        const MeshSection *sections = nullptr;
        size_t sectionCount = 0;
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
        // draw quad axis aligned
        void drawQuadAA( Vec2 origin, Vec2 halfSize, Color color );
        
        void drawConvexPolygon( const Vertex *vertexes, unsigned count );
        void drawConvexPolygon( const Vec2 *positions, unsigned count, Color color );
        void drawConvexPolygon( std::initializer_list<Vertex> vertexes );
        void drawConvexPolygon( std::initializer_list<Vec2> positions, Color color );
        
        void drawPolyLine( const PolyLineControllPoint *controllPoints, unsigned count,  PolylineOptions options = PolylineOptions());
        void drawPolyLine( const Vec2 *positions, unsigned count, Color color, float thickness, PolylineOptions options = PolylineOptions() );
        void drawPolyLine( std::initializer_list<PolyLineControllPoint> controllPoints, PolylineOptions options = PolylineOptions());
        void drawPolyLine( std::initializer_list<Vec2> positions, Color color, float thickness, PolylineOptions options = PolylineOptions() );
        
        void drawPolyLoop( const PolyLineControllPoint *controllPoints, unsigned count, PolyloopOptions options = PolyloopOptions() );
        void drawPolyLoop( const Vec2 *positions, unsigned count, Color color, float thickness, PolyloopOptions options = PolyloopOptions() );
        void drawPolyLoop( std::initializer_list<PolyLineControllPoint> controllPoints, PolyloopOptions options = PolyloopOptions() );
        void drawPolyLoop( std::initializer_list<Vec2> positions, Color color, float thickness, PolyloopOptions options = PolyloopOptions() );
        
    private:
        struct Impl;
        Impl *impl() {
            return reinterpret_cast<Impl*>(&mImpl);
        }
        const Impl* impl() const {
            return reinterpret_cast<const Impl*>(&mImpl);
        }
        
        static const int IMPL_SIZE = 128;
        std::aligned_storage<IMPL_SIZE, alignof(void*)>::type mImpl;
    };
}
