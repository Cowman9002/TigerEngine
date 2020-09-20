#include "TigerEngine/colliders.h"

#include <m3d/Math1D.h>

#include <algorithm>

namespace tgr
{
    Line::Line() : Line(m3d::vec3(), m3d::vec3()) {}
    Line::Line(m3d::vec3 start, m3d::vec3 end) :
        Collider(ColliderType::Line), m_start(start), m_end(end) {}

    m3d::vec3 Line::getStart()
    {
        return m_start;
    }

    m3d::vec3 Line::getEnd()
    {
        return m_end;
    }

    Line& Line::setStart(m3d::vec3 start)
    {
        m_start = start;
        return *this;
    }

    Line& Line::setEnd(m3d::vec3 end)
    {
        m_end = end;
        return *this;
    }

    Collision Line::checkCollision(const Collider* other) const
    {
        Collision res;

        switch(other->getType())
        {
        case ColliderType::AABB:
            {
                AABB *b = (AABB*)other;
                break;
            }
        case ColliderType::Sphere:
            {
                Sphere *b = (Sphere*)other;
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

                float m = (m_end.y - m_start.y) / (m_end.x - m_start.x);
                float k = m_start.y - m * m_start.x;

                float pass = b->getPoint().y - (m * b->getPoint().x + k);

                if (std::abs(pass) < 0.0001f)
                {
                    res.hit = true;
                }

                return res;

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

    m3d::vec3 Line::nearestPoint(const m3d::vec3& point) const
    {
        m3d::vec3 l = m_end - m_start;
        m3d::vec3 p = point - m_start;
        float d = m3d::vec3::dot(l, p) / m3d::vec3::lengthSqr(l);
        d = m3d::clamp(d, 0.0f, 1.0f);

        return m_start + l * d;
    }
}
