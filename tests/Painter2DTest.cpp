
#include "Painter2D.h"

#include <SFML/Window.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Graphics.hpp>

using namespace Painter;

void render( const Painter2D &painter, sf::RenderTarget &target, bool wireframe = false )
{
    auto mesh = painter.getMesh();
    sf::VertexArray va(wireframe ? sf::Lines : sf::Triangles, wireframe ? (2*mesh.indexCount) : mesh.indexCount);
    
    auto toSF = []( Vertex v ) -> sf::Vertex {
        return sf::Vertex(
            sf::Vector2f(v.position.x, v.position.y),
            sf::Color(v.color.r, v.color.g, v.color.b, v.color.a)
        );
        
    };
    
    if (wireframe) {
        for (size_t i=0; i < mesh.indexCount/3; ++i) {
            IndexType i0 = mesh.indexes[i*3+0],
                      i1 = mesh.indexes[i*3+1],
                      i2 = mesh.indexes[i*3+2];
            
            va[i*6+0] = toSF(mesh.vertexes[i0]);
            va[i*6+1] = toSF(mesh.vertexes[i1]);
            
            va[i*6+2] = toSF(mesh.vertexes[i1]);
            va[i*6+3] = toSF(mesh.vertexes[i2]);
            
            va[i*6+4] = toSF(mesh.vertexes[i2]);
            va[i*6+5] = toSF(mesh.vertexes[i0]);
        }
    }
    else {
        for (size_t i=0; i < mesh.indexCount; ++i) {
            auto &v = va[i];
            
            IndexType idx = mesh.indexes[i];
            v = toSF(mesh.vertexes[idx]);
        }
    }
    
    target.draw(va);
}

void saveToFile( const Painter2D &painter, int width, int height, std::string filename, bool wireframe = false )
{
    sf::RenderTexture target;
    target.create(width, height);
    
    target.clear();
    render(painter, target, wireframe);
    
    sf::Image image = target.getTexture().copyToImage(); 
    image.flipVertically();
    image.saveToFile(filename);
}

void polylineTest() {
    Painter2D painter;
    
    int sizeX = 2048,
        sizeY = 2048;
          
    int stepX = 8,
        stepY = 8;
        
    int offsetX = sizeX / stepX,
        offsetY = sizeY / stepY;
        
    int total = stepX * stepY;
    
    bool wireframe = false;
    
    for (int i=0; i < total; ++i) {
        int y = i / stepY,
            x = i % stepX;
        
        float angle = 2.f * M_PI * ((float)i / total);
        
        float dx = std::cos(angle) * 0.3f,
              dy = std::sin(angle) * 0.3f;
              
        Vec2 pos[3] = {
            {(float)offsetX * (x+0.5f), (float)offsetY * (y+0.1f)},
            {offsetX * (x+0.5f), offsetY * (y+0.5f)},
            {offsetX * (x+0.5f+dx), offsetY * (y+0.5f+dy)}
        };
        
        if ((x+y) & 1 && !wireframe) {
            painter.drawQuad({offsetX * (x+0.5f), offsetY * (y+0.5f)}, {(float)offsetX*0.5f,0.f}, {0.f,(float)offsetY*0.5f}, Color(255,255,255,100));
        }
        
        PolyLineControllPoint points[3] = {
            {pos[0], Color(255,0,0), 15.f},
            {pos[1], Color(0,255,0), 15.f},
            {pos[2], Color(0,0,255), 15.f}
        };
        painter.drawPolyLine(points, 3, {JointType::Bevel});
    }
    
    saveToFile(painter, sizeX, sizeY, "PolylineTest.png", wireframe);
}


void polyloopTest() {
    Painter2D painter;
    
    int sizeX = 2048,
        sizeY = 2048;
          
    int stepX = 8,
        stepY = 8;
        
    int offsetX = sizeX / stepX,
        offsetY = sizeY / stepY;
        
    int total = stepX * stepY;
    
    bool wireframe = false;
    
    for (int i=0; i < total; ++i) {
        int y = i / stepY,
            x = i % stepX;
        
        float angle = 2.f * M_PI * ((float)i / total);
        
        float dx = std::cos(angle) * 0.3f,
              dy = std::sin(angle) * 0.3f;
              
        Vec2 pos[3] = {
            {(float)offsetX * (x+0.5f), (float)offsetY * (y+0.1f)},
            {offsetX * (x+0.5f), offsetY * (y+0.5f)},
            {offsetX * (x+0.5f+dx), offsetY * (y+0.5f+dy)}
        };
        
        if ((x+y) & 1 && !wireframe) {
            painter.drawQuad({offsetX * (x+0.5f), offsetY * (y+0.5f)}, {(float)offsetX*0.5f,0.f}, {0.f,(float)offsetY*0.5f}, Color(255,255,255,100));
        }
        
        PolyLineControllPoint points[3] = {
            {pos[0], Color(255,0,0), 15.f},
            {pos[1], Color(0,255,0), 15.f},
            {pos[2], Color(0,0,255), 15.f}
        };
        painter.drawPolyLoop(points, 3, {JointType::Bevel});
    }
    
    saveToFile(painter, sizeX, sizeY, "PolyloopTest.png", wireframe);
}


int main()
{
    //sf::RenderWindow window(sf::VideoMode(512, 512), "Painter2DTest");
    polylineTest();
    polyloopTest();
    
}
