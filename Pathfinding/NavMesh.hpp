#pragma once
#include "Search.hpp"
#include <cmath>

struct Point {
    double x = 0.0, z = 0.0;
};
inline Point operator+(Point a, Point b) { return { a.x + b.x,a.z + b.z }; }
inline Point operator-(Point a, Point b) { return { a.x - b.x,a.z - b.z }; }
inline Point operator*(Point a, double s) { return { a.x * s,a.z * s }; }
inline Point operator/(Point a, double s) { return { a.x / s,a.z / s }; }
inline double Cross(Point a, Point b) { return a.x * b.z - a.z * b.x; }
inline double Length(Point a) { return std::hypot(a.x, a.z); }
inline double Distance(Point a, Point b) { return Length(a - b); }
inline bool Finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.z); }
inline bool SamePoint(Point a, Point b) { return Distance(a, b) < 1e-8; }

struct Polygon {
    std::vector<Point> vertices;
    Point center;
};
struct Portal {
    int a, b;
    Point left, right;
    Point Middle() const { return (left + right) * 0.5; }
};

inline Polygon MakePolygon(std::vector<Point> vertices)
{
    Polygon polygon;
    polygon.vertices = vertices;
    if (vertices.size() < 3)
        throw std::invalid_argument("pas assez de points");
    for (Point p : vertices)
    {
        if (!Finite(p))
            throw std::invalid_argument("un point n'est pas finis");
    }
    for (int i = 0; i < vertices.size(); i++)
    {
        for (int j = 0; j < i; j++)
            if (SamePoint(vertices[i], vertices[j]))
                throw std::invalid_argument("il y a 2 fois le même point");
    }
    for (int i = 0; i < vertices.size(); i++)
        polygon.center = polygon.center + vertices[i];
    polygon.center = polygon.center / vertices.size();
    return polygon;
}


inline bool Contains(const Polygon& polygon, Point point)
{
    Point a;
    Point b;
    double side;
    bool positif = false;
    bool negatif = false;
    if (!Finite(point))
        throw std::invalid_argument("un point n'est pas finis");
    for (int i = 0; i < polygon.vertices.size(); i++)
    {
        a = polygon.vertices[i];
        b = polygon.vertices[(i + 1) % polygon.vertices.size()];
        side = Cross(b - a, point - a);
        std::cout << "side" << side << std::endl;
        if (side < -1e-8)
            negatif = true;
        else if (side > 1e-8)
            positif = true;
    }
    return !(positif && negatif);
}

struct NavMesh {
    std::vector<Polygon> polygons;
    std::vector<Portal> portals;
    Graph graph;

    int Locate(Point p) const
    {
        if (!Finite(p))
            return -1;
        for (int i = 0; i < polygons.size(); i++)
        {
            if (Contains(polygons[i], p))
                return i;
        }
        return -1;
    }
    const Portal* Between(int a, int b) const;
    void BuildGraph();
};

struct NavPath {
    SearchResult search;
    std::vector<Point> points;
    double routeCost = std::numeric_limits<double>::infinity();
};

inline NavMesh MakeDemoMesh();

inline NavPath FindNavPath(const NavMesh& mesh, Point start,
    Point goal, bool useAStar = true); // Étape 7





























//const Portal* Between(int a, int b) const
//{
//    for (Portal p : portals)
//    {
//        if ((p.a == a && p.b == b) || (p.a == b && p.b == a))
//            return &p;
//    }
//    return nullptr;
//}
//void BuildGraph()
//{
//    Point middle;
//    double cost;
//    graph.clear();
//    graph.resize(polygons.size());
//    for (Portal p : portals)
//    {
//        if (p.a < 0 || p.b < 0 || p.a >= polygons.size() || p.b >= polygons.size() || p.a == p.b)
//            throw std::invalid_argument("indice de portail invalides");
//        if (Distance(p.left, p.right) < 1e-8)
//            throw std::invalid_argument("longueur nulle");
//        if (!Contains(polygons[p.a], p.left) || !Contains(polygons[p.b], p.right))
//            throw std::invalid_argument("extremite n'est pas dans la bonne region");
//        middle = p.Middle();
//        cost = Distance(polygons[p.a].center, middle) + Distance(middle, polygons[p.b].center);
//        AddUndirected(graph, p.a, p.b, cost);
//    }
//}
//inline NavMesh MakeDemoMesh()
//{
//    NavMesh mesh;
//    mesh.polygons = {
//        MakePolygon({ { 0,2 },{ 4,2 },{ 4,0 },{ 0,0 } }),
//        MakePolygon({ { 4,2 },{ 6,2 },{ 6,0 },{ 4,0 } }),
//        MakePolygon({ { 4,6 },{ 6,6 },{ 6,2 },{ 4,2 } }),
//        MakePolygon({ { 8,2 },{ 10,2 },{ 10,0 },{ 8,0 } })
//    };
//    mesh.portals = {
//    { 0, 1,{ 4,0 },{ 4,2 } },
//    { 1, 2,{ 4,2 },{ 6,2 } }
//    };
//    mesh.BuildGraph();
//    return mesh;
//}