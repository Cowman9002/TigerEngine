#include "TigerEngine/colliders.h"

#include <algorithm>

namespace tgr
{
    Sphere::Sphere() : Sphere(m3d::vec3(), 0.0f) {}

    Sphere::Sphere(const m3d::vec3& center, const float& radius) :
        Collider(ColliderType::Sphere), m_center(center), m_radius(radius) {}

    Sphere& Sphere::genFromPoints(const m3d::vec3 *points, unsigned long int num_points)
    {
        if(num_points < 1) return *this;

        m3d::vec3 sum;
        float d = 0.0f;

        for(unsigned i = 0; i < num_points; i++)
        {
            sum += points[i];

            // loop over all previous points to find the max distance between any two points
            for(unsigned j = 0; j < i; j++)
            {
                float dist = m3d::vec3::distance(points[i], points[j]);
                d = std::max(dist, d);
            }
        }

        m_radius = d / 2.0f;
        m_center = sum / num_points;

        return *this;
    }

    Sphere& Sphere::genFromPoints(const std::vector<m3d::vec3>& points)
    {
        return genFromPoints(points.data(), points.size());
    }

    m3d::vec3 Sphere::getCenter() const
    {
        return m_center;
    }

    float Sphere::getRadius() const
    {
        return m_radius;
    }

    Sphere& Sphere::setCenter(const m3d::vec3& new_center)
    {
        m_center = new_center;
        return *this;
    }

    Sphere& Sphere::setRadius(const float& new_radius)
    {
        m_radius = new_radius;
        return *this;
    }

    Sphere& Sphere::translate(const m3d::vec3& translation)
    {
        m_center += translation;
        return *this;
    }

    Collision Sphere::checkCollision(const Collider* other) const
    {
        Collision res;

        switch(other->getType())
        {
        case ColliderType::AABB:
            {
                AABB *b = (AABB*)other;

                res = b->checkCollision(this);

                break;
            }
        case ColliderType::Sphere:
            {
                Sphere *b = (Sphere*)other;

                float dist = m3d::vec3::distance(m_center, b->getCenter());
                float sumr = m_radius + b->getRadius();

                res.hit = dist <= sumr;
                res.penetration = sumr - dist;
                res.normal = (m_center - b->getCenter()).normalized();

                break;
            }
        case ColliderType::Plane:
            {
                Plane *b = (Plane*)other;

                res.hit = m3d::vec3::distance(b->nearestPoint(m_center), m_center) <= m_radius;

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

    m3d::vec3 Sphere::nearestPoint(const m3d::vec3& point) const
    {
        return m_center + (point - m_center).normalized() * m_radius;
    }
}
