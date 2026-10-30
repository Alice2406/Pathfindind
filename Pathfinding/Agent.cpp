#include "Agent.hpp"
#include <algorithm>
#include <stdexcept>
#include <algorithm>
#include <utility>

bool SegmentInside(const NavMesh& mesh, Point a, Point b) {
    if (!Finite(a) || !Finite(b)) return false;
    std::vector<std::pair<double, double>> intervals;
    const Point direction = b - a;
    for (const Polygon& polygon : mesh.polygons) {
        const auto& v = polygon.vertices;
        double orientation = Cross(v[1] - v[0], v[2] - v[1]) > 0 ? 1.0 : -1.0;
        double low = 0.0, high = 1.0;
        bool valid = true;
        for (std::size_t i = 0; i < v.size(); ++i) {
            Point edge = v[(i + 1) % v.size()] - v[i];
            double base = orientation * Cross(edge, a - v[i]);
            double slope = orientation * Cross(edge, direction);
            // base + slope*t >= -epsilon
            if (std::abs(slope) < 1e-12) {
                if (base < -1e-9) { valid = false; break; }
            }
            else {
                double bound = (-1e-9 - base) / slope;
                if (slope > 0) low = std::max(low, bound);
                else high = std::min(high, bound);
            }
        }
        if (valid && low <= high) intervals.push_back({ low,high });
    }
    std::sort(intervals.begin(), intervals.end());
    double covered = 0.0;
    for (auto [low, high] : intervals) {
        if (low > covered + 1e-8) return false;
        covered = std::max(covered, high);
        if (covered >= 1.0 - 1e-8) return true;
    }
    return false;
}

const char* StateName(EnemyState state) {
    switch (state) {
    case EnemyState::Idle: return "Idle";
    case EnemyState::Chase: return "Chase";
    case EnemyState::Windup: return "Windup";
    case EnemyState::Recover: return "Recover";
    case EnemyState::Dead: return "Dead";
    }
    return "Unknown";
}

void NavAgent::Tick(double dt) { requestCooldown = std::max(0.0, requestCooldown - dt); }

void NavAgent::Clear() { path = NavPath{}; next = 0; hasGoal = false; }

bool NavAgent::Finished() const { return next >= path.points.size(); }

void NavAgent::Plan(const NavMesh& mesh, Point goal) {
    if (requestCooldown > 0)
        return;
    if (!hasGoal || Distance(lastGoal, goal) > 0.75 || path.search.status != Status::Found || (Finished() && Distance(position, goal) > 0.1))
    {
        path = FindNavPath(mesh, position, goal, useAStar);
        hasGoal = true;
        lastGoal = goal;
        next = path.points.size() > 1 ? 1 : path.points.size();
        requestCooldown = 0.3;
        requests++;
    }
}

void NavAgent::Follow(double dt) {
    if (path.search.status != Status::Found || dt <= 0 || speed <= 0) return;
    double budget = speed * dt;
    while (!Finished() && budget > 0) {
        Point delta = path.points[next] - position;
        double distance = Length(delta);
        if (distance < 1e-8) { ++next; continue; }
        if (distance <= budget) {
            position = path.points[next++];
            budget -= distance;
        }
        else {
            position = position + delta * (budget / distance);
            budget = 0;
        }
    }
}

void Enemy::Update(const NavMesh& mesh, Target& target, double dt) {
    // Contrat : dt fini et >=0, positions initiales dans le mesh.
    nav.Tick(dt);
    if (hp <= 0) { state = EnemyState::Dead; nav.Clear(); return; }
    if (state == EnemyState::Dead) return;
    const double distance = Distance(nav.position, target.position);
    if (target.hp <= 0 || distance > loseRange) {
        state = EnemyState::Idle; nav.Clear(); timer = 0; return;
    }
    switch (state) {
    case EnemyState::Idle:
        if (distance <= detectRange) state = EnemyState::Chase;
        break;
    case EnemyState::Chase:
        if (distance <= attackRange && SegmentInside(mesh, nav.position, target.position)) {
            state = EnemyState::Windup;
            timer = windupDuration;
        }
        else {
            nav.Plan(mesh, target.position);
            nav.Follow(dt);
        }
        break;
    case EnemyState::Windup:
        timer -= dt;
        if (timer <= 0) {
            if (distance <= attackRange && SegmentInside(mesh, nav.position, target.position))
                target.hp = std::max(0, target.hp - damage);
            state = EnemyState::Recover;
            timer = recoverDuration;
        }
        break;
    case EnemyState::Recover:
        timer -= dt;
        if (timer <= 0) state = EnemyState::Chase;
        break;
    case EnemyState::Dead: break;
    }
}