#include "TigerEngine/colliders.h"

#include <algorithm>

#include <stdio.h>

namespace tgr
{
    Plane::Plane() : Plane(m3d::vec3(), 0.0f) {}

    Plane::Plane(const m3d::vec3& normal, const float& distance) :
        Collider(ColliderType::Plane), m_normal(normal), m_distance(distance) {}

    Plane& Plane::genFromPoints(const m3d::vec3& p0, const m3d::vec3& p1, const m3d::vec3& p2)
    {
        m_normal = m3d::vec3::cross(p1 - p0, p2 - p0).normalized();
        m_distance = m3d::vec3::dot(m_normal, p0);
        return *this;
    }

    m3d::vec3 Plane::getNormal() const
    {
        return m_normal;
    }

    float Plane::getDistance() const
    {
        return m_distance;
    }

    Plane& Plane::setNormal(const m3d::vec3& normal)
    {
        m_normal = normal;
        return *this;
    }

    Plane& Plane::setDistance(const float& distance)
    {
        m_distance = distance;
        return *this;
    }

    Plane& Plane::translate(const float& translation)
    {
        m_distance += translation;
        return *this;
    }

    float Plane::distanceFrom(const m3d::vec3& point) const
    {
        return m3d::vec3::dot(point, m_normal) - m_distance;
    }

    Collision Plane::checkCollision(const Collider* other) const
    {
        Collision res;

        switch(other->getType())
        {
        case ColliderType::AABB:
            {
                AABB *b = (AABB*)other;
                res = b->checkCollision(this);

                res.normal *= -1.0f;

                break;
            }
        case ColliderType::Sphere:
            {
                Sphere *b = (Sphere*)other;
                res = b->checkCollision(this);
                res.normal *= -1.0f;

                break;
            }
        case ColliderType::Plane:
            {
                Plane *b = (Plane*)other;

                // cross means the length nears zero the closer the values are together
                m3d::vec3 d = m3d::vec3::cross(b->getNormal(), m_normal);
                // check if planes are parallel, if not, they collide
                if(d.lengthSqr() > 0.0001f)
                {
                    res.hit = true;
                }

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

                float dist = distanceFrom(b->getPoint());

                if(std::abs(dist) < 0.001f)
                {
                    res.hit = true;
                }

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

    m3d::vec3 Plane::nearestPoint(const m3d::vec3& point) const
    {
        return point - m_normal * distanceFrom(point);
    }
}
