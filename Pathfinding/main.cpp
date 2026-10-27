#include <iostream>
#include "NavMesh.hpp"

int main()
{
    std::vector<Point> points = { { 0, 0 },{ 4,0 },{ 4, 4 },{ 0, 4 } };
    Polygon polygon;
    polygon = MakePolygon(points);
    std::cout << polygon.center.x << "," << polygon.center.z << std::endl;
    std::cout << Contains(polygon, { 1, 1 }) << std::endl;
    std::cout << Contains(polygon, { 14, 8 }) << std::endl;
    std::cout << Contains(polygon, { 0, 2 }) << std::endl;
    std::cout << Contains(polygon, { 0, 0 }) << std::endl;
}