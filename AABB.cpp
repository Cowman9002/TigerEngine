#include "colliders.h"

#include <algorithm>

namespace tgr
{
    AABB::AABB() : AABB(m3d::vec3(), m3d::vec3()) {}

    AABB::AABB(const m3d::vec3& max_bounds, const m3d::vec3& min_bounds) :
        Collider(ColliderType::AABB), max_bounds(max_bounds), min_bounds(min_bounds) {}

    AABB& AABB::genFromPoints(const m3d::vec3 *points, unsigned long int num_points)
    {
        if(num_points < 1) return *this;

        min_bounds = points[0];
        max_bounds = points[0];

        for(unsigned i = 0; i < num_points; i++)
        {
            min_bounds = m3d::vec3::min(min_bounds, points[i]);
            max_bounds = m3d::vec3::max(max_bounds, points[i]);
        }

        return *this;
    }

    AABB& AABB::genFromPoints(const std::vector<m3d::vec3>& points)
    {
        return genFromPoints(points.data(), points.size());
    }

    m3d::vec3 AABB::getCenter() const
    {
        return (max_bounds + min_bounds) / 2.0;
    }

    m3d::vec3 AABB::getExtents() const
    {
        return (max_bounds - min_bounds) / 2.0;
    }

    m3d::vec3 AABB::getMaxBounds() const
    {
        return max_bounds;
    }

    m3d::vec3 AABB::getMinBounds() const
    {
        return min_bounds;
    }

    AABB& AABB::setMaxBounds(const m3d::vec3& new_max)
    {
        max_bounds = new_max;

        return *this;
    }

    AABB& AABB::setMinBounds(const m3d::vec3& new_min)
    {
        min_bounds = new_min;

        return *this;
    }

    AABB& AABB::setCenterExtents(const m3d::vec3& center, const m3d::vec3& extents)
    {
        max_bounds = center + extents;
        min_bounds = center - extents;

        return *this;
    }

    AABB& AABB::normalize()
    {
        min_bounds = m3d::vec3::min(min_bounds, max_bounds);
        max_bounds = m3d::vec3::max(min_bounds, max_bounds);

        return *this;
    }

    AABB& AABB::translate(const m3d::vec3& translation)
    {
        min_bounds += translation;
        max_bounds += translation;

        return *this;
    }

    Collision AABB::checkCollision(const Collider* other) const
    {
        Collision res;

        switch(other->getType())
        {
        case ColliderType::AABB:
            {
                AABB *b = (AABB*)other;

                if(min_bounds.x <= b->getMaxBounds().x && max_bounds.x >= b->getMinBounds().x &&
                        min_bounds.y <= b->getMaxBounds().y && max_bounds.y >= b->getMinBounds().y &&
                        min_bounds.z <= b->getMaxBounds().z && max_bounds.z >= b->getMinBounds().z)
                {
                    res.hit = true;
                }

                break;
            }
        case ColliderType::Sphere:
            {
                Sphere *b = (Sphere*)other;

                res.hit = m3d::vec3::distance(nearestPoint(b->getCenter()), b->getCenter()) <= b->getRadius();

                break;
            }
        case ColliderType::Plane:
            {
                Plane *b = (Plane*)other;
                break;
            }
        case ColliderType::Triangle:
            {
                Triangle *b = (Triangle*)other;
                break;
            }
        case ColliderType::OBB:
            {
                OBB *b = (OBB*)other;
                break;
            }
        case ColliderType::Capsule:
            {
                Capsule *b = (Capsule*)other;
                break;
            }
        case ColliderType::Point:
            {
                Point *b = (Point*)other;
                break;
            }
        case ColliderType::Line:
            {
                Line *b = (Line*)other;
                break;
            }
        case ColliderType::Ray:
            {
                Ray *b = (Ray*)other;
                break;
            }
        }

        return res;
    }

    m3d::vec3 AABB::nearestPoint(const m3d::vec3& point) const
    {
        return m3d::vec3::max(min_bounds, m3d::vec3::min(point, max_bounds));
    }
}
