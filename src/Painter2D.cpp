#include "Painter2D.h"

#include <new>
#include <vector>
#include <limits>
#include <cassert>

namespace Painter
{
    namespace internal
    {
        struct Painter2D {
            std::vector<Vertex> vertexes;
            std::vector<IndexType> indexes;
        };
        
        template< int MaxSize, int MaxAlign, int Size, int Align >
        struct CheckImpl {
            static_assert (Size <= MaxSize, "Increase IMPL_SIZE!");
            static_assert (Align <= MaxAlign, "To small alignment");
        };
        
        Mesh get_mesh( const Painter2D *impl )
        {
            Mesh mesh;
            mesh.vertexes = impl->vertexes.data();
            mesh.vertexCount = impl->vertexes.size();
            
            mesh.indexes = impl->indexes.data();
            mesh.indexCount = impl->indexes.size();
            
            return mesh;
        }
        
        void reserve_indexes( Painter2D *impl, unsigned count )
        {
        }
        
        IndexType push_vertexes( Painter2D *impl, const Vertex *vertexes, unsigned count )
        {
            size_t base = impl->vertexes.size();
            
            impl->vertexes.insert(impl->vertexes.end(), vertexes, vertexes+count);
            
            assert ((base + count) < std::numeric_limits<IndexType>::max());
            return base;
        }
        
        IndexType push_vertexes( Painter2D *impl, const Vertex *vertexes, unsigned vertexCount, const IndexType *indexes, unsigned count )
        {
            IndexType base = push_vertexes(impl, vertexes, vertexCount);
            
            reserve_indexes(impl, count);
            for (unsigned i=0; i < count; ++i) {
                impl->indexes.push_back(base + indexes[i]);
            }
            
            return base;
        }
        
        void draw_quad( Painter2D *impl, Vec2 origin, Vec2 v1, Vec2 v2, Color color )
        {
            //      v1
            //    0 --- 1
            // v2 |  \  |
            //    2 --- 3
            Vertex vertexes[4] = {
                {origin - v1 - v2, color},
                {origin + v1 - v2, color},
                {origin - v1 + v2, color},
                {origin + v1 + v2, color}
            };
            IndexType indexes[6] = {
                0, 1, 3,
                0, 3, 2
            };
            push_vertexes(impl, vertexes, 4, indexes, 6);
        }
        
        void draw_convex_polygon( Painter2D *impl, const Vertex *vertexes, unsigned count )
        {
            IndexType base = push_vertexes(impl, vertexes, count);
            
            unsigned triangleCount = count - 2;
            unsigned indexCount = triangleCount * 3;
            
            reserve_indexes(impl, indexCount);
            for (unsigned i=0; i < triangleCount; ++i) {
                impl->indexes.push_back(base + 0);
                impl->indexes.push_back(base + 1 + i);
                impl->indexes.push_back(base + 2 + i);
            }
        }
        
        
        void draw_segment( Painter2D *impl, PolyLineControllPoint p1, PolyLineControllPoint p2, CapType cap1, CapType cap2 )
        {
            Vec2 dir = p1.pos - p2.pos;
            if (isZero(dir)) return;
            
            dir = perp(normalize(dir));
            
            // 0 --- 2
            // |  /  |
            // 1 --- 3
            Vertex v[4] = {
                {p1.pos + dir * (p1.weight * 0.5f), p1.color},
                {p1.pos - dir * (p1.weight * 0.5f), p1.color},
                {p2.pos + dir * (p2.weight * 0.5f), p2.color},
                {p2.pos - dir * (p2.weight * 0.5f), p2.color}
            };
            IndexType indexes[6] = {
                0, 2, 1,
                1, 2, 3
            };
            
            push_vertexes(impl, v, 4, indexes, 6);
        }
        
        
        // Draw a cap on p1
        void draw_cap( Painter2D *impl, Vec2 pos, Vec2 dir, CapType cap ) {
            /*
            if (cap == CapType::Butt) return;
            
            Vec2 dir = p1.pos - p2.pos;
            if (isZero(dir)) return;
            
            switch (cap) {
            case CapType::Square:
                
                break;
                
            }*/
        }
        
        /*          
            DegenClass: None
                S1[0] X -- P1 -- X S1[1]
                      |          |
                      |          | 
                      |          | C[0]     S2[2]
                      |          +------------X 
                      |                       |
                S1[2] X -- P2                 P3
                      :    |                  |
                      +....X------------------X
                 C[1]     S2[1]              S2[3]
                 
            DegenClass: Fold       
                              /--P3--/
                S1[0] X -- P1/-X    /
                      |     /  :   /
                      |    /   :  /
                      |   /    : /
                      |  /     :/
                S1[2] X -- P2--X
                
            DegenClass: Straight
                              
                      X -- P1 -- X
                      |          |
                      |          |
                      X -- P2 -- X
                      |          |
                      |          |
                      X -- P3 -- X
         */

        struct PolylineAchor {
            PolyLineControllPoint P1, P2, P3;
            Vec2 d1, d2;
            
            float cos_a, sin_a;
            
        };
        
        struct PolyLineMitorInfo {
            IndexType v2, v4;
            Vec2 bevelVec;
        };
        
        PolyLineMitorInfo polyline_achor_miter( Painter2D *impl, const PolylineAchor &achor ) {
            /* 
                0 -P1- 1
                |      |
                |      3-----5
                2            |
                 \  p2       p3
                  \          |
                   4---------6
             */
            
            Vec2 p1 = achor.P1.pos,
                 p2 = achor.P2.pos,
                 p3 = achor.P3.pos;
            Color c1 = achor.P1.color,
                  c2 = achor.P2.color,
                  c3 = achor.P3.color;
            Vec2 d1 = perp(achor.d1 * achor.P1.weight),
                 d2 = perp(achor.d1 * achor.P2.weight),
                 d3 = perp(achor.d2 * achor.P2.weight),
                 d4 = perp(achor.d2 * achor.P3.weight);
            
            Vec2 halfDir = normalize(achor.d1 - achor.d2);
            
            float cos_a2 = dot(halfDir, achor.d1);
            float sin_a2 = std::sqrt(1 - cos_a2*cos_a2);
            float d = achor.P2.weight / sin_a2;
            
            Vec2 d5 = halfDir * d;
            
            Vertex vertexes[7] = {
                {p1 + d1, c1},
                {p1 - d1, c1},
                {p2 + d2, c2},
                {p2 - d5, c2},
                {p2 + d3, c2},
                {p3 - d4, c3},
                {p3 + d4, c3}
            };
            IndexType indexes[15] = {
                0, 1, 2,
                2, 1, 3,
                2, 3, 4,
                4, 3, 5,
                4, 5, 6
            };
            
            IndexType base = push_vertexes(impl, vertexes, 7, indexes, 15);
            
            PolyLineMitorInfo info;
                info.v2 = base + 2;
                info.v4 = base + 4;
                info.bevelVec = d5 * -1.f;
                
            return info;
        }
        
        void polyline_achor_bevel( Painter2D *impl, const PolylineAchor &achor ) {
            /* 
                0 -P1- 1
                |      |
                |      3-----5
                2            |
                |   p2       p3
                |            |
                X--4---------6
             */
            
            PolyLineMitorInfo miter = polyline_achor_miter(impl, achor);
            
            Vertex bevelVert = {
                achor.P2.pos + miter.bevelVec,
                achor.P2.color
            };
            
            IndexType bevelIdx = push_vertexes(impl, &bevelVert, 1);
            IndexType indexes[3] = {
                bevelIdx,
                miter.v2, 
                miter.v4
            };
            
            impl->indexes.insert(impl->indexes.end(), std::begin(indexes), std::end(indexes));
        }
        
        void polyline_achor_round( Painter2D *impl, const PolylineAchor &achor ) {
            polyline_achor_miter(impl, achor);
            
            /// @todo add round joint
        }
         
        // Degenerative case straight
        void polyline_achor_straight( Painter2D *impl, const PolylineAchor &achor ) {
            /*  0--P1--1
                |      |
                2--P2--3
                |      |
                4--P3--5
             */
            
            Vec2 d1 = perp(achor.d1),
                 d3 = perp(achor.d2);
            Vec2 d2 = normalize(d1 + d3);
            
            Vec2 p1 = achor.P1.pos,
                 p2 = achor.P2.pos,
                 p3 = achor.P3.pos;
                 
            Color c1 = achor.P1.color,
                  c2 = achor.P2.color,
                  c3 = achor.P3.color;
                  
            float w1 = achor.P1.weight,
                  w2 = achor.P2.weight,
                  w3 = achor.P3.weight;
            
            
            Vertex vertexes[6] = {
                {p1 - d1*w1, c1},
                {p1 + d1*w2, c1},
                {p2 - d2*w2, c2},
                {p2 + d2*w2, c2},
                {p3 - d3*w3, c3},
                {p3 + d3*w3, c3}
            };
            IndexType indexes[12] = {
                0, 1, 2,
                2, 1, 3,
                2, 3, 4,
                4, 3, 5
            };
            
            push_vertexes(impl, vertexes, 6, indexes, 12);
        }
        
        // Degenerative case fold
        void polyline_achor_fold( Painter2D *impl, const PolylineAchor &achor ) {
            /*  4--P3--5
                |      |
                0--P1--1
                |      |
                2--P2--3
             */
            
            Vec2 d1 = perp(achor.d1),
                 d3 = perp(achor.d2) * -1.f;
            Vec2 d2 = normalize(d1 + d3);
            
            Vec2 p1 = achor.P1.pos,
                 p2 = achor.P2.pos,
                 p3 = achor.P3.pos;
                 
            Color c1 = achor.P1.color,
                  c2 = achor.P2.color,
                  c3 = achor.P3.color;
                  
            float w1 = achor.P1.weight,
                  w2 = achor.P2.weight,
                  w3 = achor.P3.weight;
            
            
            Vertex vertexes[6] = {
                {p1 - d1*w1, c1},
                {p1 + d1*w2, c1},
                {p2 - d2*w2, c2},
                {p2 + d2*w2, c2},
                {p3 - d3*w3, c3},
                {p3 + d3*w3, c3}
            };
            IndexType indexes[12] = {
                0, 1, 2,
                2, 1, 3,
                2, 3, 4,
                4, 3, 5
            };
            
            push_vertexes(impl, vertexes, 6, indexes, 12);
        }
        
        // Degenerative case fold with round joint
        void polyline_achor_fold_round( Painter2D *impl, const PolylineAchor &achor ) {
            polyline_achor_fold(impl, achor);
            
            /// @todo add round joint
        }
        
        void polyline_achor( Painter2D *impl, PolyLineControllPoint P1, PolyLineControllPoint P2, PolyLineControllPoint P3, JointType joint ) 
        {
            Vec2 d1 = normalize(P2.pos - P1.pos),
                 d2 = normalize(P3.pos - P2.pos);
                 
            float cos_a = dot(d1, d2),
                  sin_a = cross(d1, d2);
            

            
            PolylineAchor achor;
            achor.P1 = P1;
            achor.P2 = P2;
            achor.P3 = P3;
            achor.d1 = d1;
            achor.d2 = d2;
            achor.cos_a = cos_a;
            achor.sin_a = sin_a;
            
            
            const float sin_inner_degen = 0.3420f, // sin(20 degrees)
                        sin_outer_degen = 0.5f; // sin(30 degrees)
            
            if (joint == JointType::Bevel && sin_a < sin_outer_degen) {
                joint = JointType::Miter;
            }
            
            // check our degenerative cases
            if (std::abs(sin_a) < sin_inner_degen) {
                if (cos_a < 0.f) {
                    if (joint == JointType::Round) {
                        return polyline_achor_fold_round(impl, achor);
                    }
                    else {
                        return polyline_achor_fold(impl, achor);
                    }
                }   
                else {
                    return polyline_achor_straight(impl, achor);
                }
            }
            // By only handling the bend in one direction it makes things easier down the drawPolyLine
            //  if its the wrong way just flip the direction
            if (sin_a < 0.f) {
                using std::swap;
                swap(achor.P1, achor.P3);
                std::swap(achor.d1, achor.d2);
                achor.d1 = achor.d1 * -1.f;
                achor.d2 = achor.d2 * -1.f;
                achor.sin_a = achor.sin_a * -1.f;
            }
            
            switch (joint) {
            case JointType::Bevel:
                return polyline_achor_bevel(impl, achor);
            case JointType::Miter:
                return (void)polyline_achor_miter(impl, achor);
            case JointType::Round:
                return polyline_achor_round(impl, achor);
            }
        }
        
        
        
        void draw_polyline( Painter2D *impl, const PolyLineControllPoint *points, unsigned count, PolylineOptions options )
        {
            if (count < 2) return;
            if (count == 2) {
                return draw_segment(impl, points[0], points[1], options.startCap, options.endCap);
            }
            
            /// @todo caps
            for (int i=2; i < count; ++i) {
                PolyLineControllPoint p1 = points[i-2];
                PolyLineControllPoint p2 = points[i-1];
                PolyLineControllPoint p3 = points[i];
                
                if (i != 2) {
                    p1.pos = (p1.pos + p2.pos) * 0.5f;
                    p1.color = mix(p1.color, p2.color);
                    p1.weight = (p1.weight + p2.weight)*0.5F;
                }
                if (i != (count-1)) {
                    p3.pos = (p2.pos + p3.pos) * 0.5f;
                    p3.color = mix(p2.color, p3.color);
                    p3.weight = (p2.weight + p3.weight)*0.5F;
                }
                
                polyline_achor(impl, p1, p2, p3, options.joint);
            }
        }
        
        void draw_polyloop( Painter2D *impl, const PolyLineControllPoint *points, unsigned count, PolyloopOptions options )
        {
            if (count < 2) return;
            if (count == 2) {
                return draw_segment(impl, points[0], points[1], CapType::Butt, CapType::Butt);
            }
            
            for (int i=0; i < count; ++i) {
                PolyLineControllPoint p1 = points[((i-2) + count) % count];
                PolyLineControllPoint p2 = points[((i-1) + count) % count];
                PolyLineControllPoint p3 = points[i];
            
                p1.pos = (p1.pos + p2.pos) * 0.5f;
                p1.color = mix(p1.color, p2.color);
                p1.weight = (p1.weight + p2.weight)*0.5F;
                
                p3.pos = (p2.pos + p3.pos) * 0.5f;
                p3.color = mix(p2.color, p3.color);
                p3.weight = (p2.weight + p3.weight)*0.5F;
                
                polyline_achor(impl, p1, p2, p3, options.joint);
            }
        }
    }
    
    /// @todo figure out the optimal value
    static const int SMALL_VECTOR_OPTIMIZATION = 8;
    
    
    struct Painter2D::Impl :
        public internal::Painter2D
    {};
    

    Painter2D::Painter2D()
    {
        internal::CheckImpl<IMPL_SIZE, alignof(mImpl), sizeof(Impl), alignof(Impl)>();
        new (impl()) Impl;
    }
    
    Painter2D::~Painter2D()
    {
        impl()->~Impl();
    }
    
    Mesh Painter2D::getMesh() const
    {
        return internal::get_mesh(impl());
    }
    
    void Painter::Painter2D::drawQuad( Vec2 origin, Vec2 v1, Vec2 v2, Color color )
    {
        return internal::draw_quad(impl(), origin, v1, v2, color);
    }
    
    void Painter2D::drawConvexPolygon( const Vertex *vertexes, unsigned count )
    {
        return internal::draw_convex_polygon(impl(), vertexes, count);
    }
    
    void Painter2D::drawConvexPolygon( const Vec2 *positions, unsigned count, Color color )
    {
        // If we only have a small number of positions, allocate the vertexes on the stack
        if (count < SMALL_VECTOR_OPTIMIZATION) {
            Vertex vertexes[SMALL_VECTOR_OPTIMIZATION];
            for (unsigned i=0; i < count; ++i) {
                vertexes[i] = Vertex{positions[i], color};
            }
            
            return drawConvexPolygon(vertexes, count);
        }
        
        std::vector<Vertex> vertexes;
        vertexes.reserve(count);
        
        for (unsigned i=0; i < count; ++i) {
            vertexes.emplace_back( Vertex{positions[i], color});
        }
        
        return drawConvexPolygon(vertexes.data(), vertexes.size());
    }

    void Painter2D::drawPolyLine( const PolyLineControllPoint *controllPoints, unsigned count, PolylineOptions options )
    {
        return internal::draw_polyline(impl(), controllPoints, count, options);
    }
    
    void Painter2D::drawPolyLine( const Vec2 *positions, unsigned count, Color color, float thickness,PolylineOptions options )
    {
        if (count < SMALL_VECTOR_OPTIMIZATION) {
            PolyLineControllPoint controllPoints[SMALL_VECTOR_OPTIMIZATION];
            for (unsigned i=0; i < count; ++i) {
                controllPoints[i] = PolyLineControllPoint{positions[i], color, thickness};
            }
            return drawPolyLine(controllPoints, count, options);
        }
        
        std::vector<PolyLineControllPoint> controllPoints;
        controllPoints.reserve(count);
        
        for (unsigned i=0; i < count; ++i) {
            controllPoints.push_back(PolyLineControllPoint{positions[i], color, thickness});
        }
        
        return drawPolyLine(controllPoints.data(), count, options);
    }
    
    void Painter2D::drawPolyLoop(const Painter::PolyLineControllPoint* controllPoints, unsigned int count, Painter::PolyloopOptions options)
    {
        return internal::draw_polyloop(impl(), controllPoints, count, options);
    }

    void Painter::Painter2D::drawPolyLoop(const Painter::Vec2* positions, unsigned int count, Painter::Color color, float thickness, Painter::PolyloopOptions options)
    {
        if (count < SMALL_VECTOR_OPTIMIZATION) {
            PolyLineControllPoint controllPoints[SMALL_VECTOR_OPTIMIZATION];
            for (unsigned i=0; i < count; ++i) {
                controllPoints[i] = PolyLineControllPoint{positions[i], color, thickness};
            }
            return drawPolyLoop(controllPoints, count, options);
            
        }
        
        std::vector<PolyLineControllPoint> controllPoints;
        controllPoints.reserve(count);
        
        for (unsigned i=0; i < count; ++i) {
            controllPoints.push_back(PolyLineControllPoint{positions[i], color, thickness});
        }
        
        return drawPolyLoop(controllPoints.data(), count, options);
    }
}



