#include "TigerEngine/colliders.h"

#include <algorithm>

namespace tgr
{
    AABB::AABB() : AABB(m3d::vec3(), m3d::vec3()) {}

    AABB::AABB(const m3d::vec3& max_bounds, const m3d::vec3& min_bounds) :
        Collider(ColliderType::AABB), m_max(max_bounds), m_min(min_bounds) {}

    AABB& AABB::genFromPoints(const m3d::vec3 *points, unsigned long int num_points)
    {
        if(num_points < 1) return *this;

        m_min = points[0];
        m_max = points[0];

        for(unsigned i = 0; i < num_points; i++)
        {
            m_min = m3d::vec3::min(m_min, points[i]);
            m_max = m3d::vec3::max(m_max, points[i]);
        }

        return *this;
    }

    AABB& AABB::genFromPoints(const std::vector<m3d::vec3>& points)
    {
        return genFromPoints(points.data(), points.size());
    }

    m3d::vec3 AABB::getCenter() const
    {
        return (m_max + m_min) / 2.0;
    }

    m3d::vec3 AABB::getExtents() const
    {
        return (m_max - m_min) / 2.0;
    }

    m3d::vec3 AABB::getMax() const
    {
        return m_max;
    }

    m3d::vec3 AABB::getMin() const
    {
        return m_min;
    }

    AABB& AABB::setMax(const m3d::vec3& new_max)
    {
        m_max = new_max;

        return *this;
    }

    AABB& AABB::setMin(const m3d::vec3& new_min)
    {
        m_min = new_min;

        return *this;
    }

    AABB& AABB::setCenterExtents(const m3d::vec3& center, const m3d::vec3& extents)
    {
        m_max = center + extents;
        m_min = center - extents;

        return *this;
    }

    AABB& AABB::normalize()
    {
        m_min = m3d::vec3::min(m_min, m_max);
        m_max = m3d::vec3::max(m_min, m_max);

        return *this;
    }

    AABB& AABB::translate(const m3d::vec3& translation)
    {
        m_min += translation;
        m_max += translation;

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

                if(m_min.x <= b->getMax().x && m_max.x >= b->getMin().x &&
                        m_min.y <= b->getMax().y && m_max.y >= b->getMin().y &&
                        m_min.z <= b->getMax().z && m_max.z >= b->getMin().z)
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

                // Convert AABB to center-extents representation
                m3d::vec3 c = getCenter();
                m3d::vec3 e = getExtents();

                // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
                float r = m3d::vec3::dot(e, m3d::vec3(
                                    std::abs(b->getNormal().x),
                                    std::abs(b->getNormal().y),
                                    std::abs(b->getNormal().z)));

                // Compute distance of box center from plane
                float s = m3d::vec3::dot(b->getNormal(), c) - b->getDistance();

                // Intersection occurs when distance s falls within [-r,+r] interval
                res.hit = std::abs(s) <= r;

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
        return m3d::vec3::max(m_min, m3d::vec3::min(point, m_max));
    }
}
