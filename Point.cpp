#include "TigerEngine/colliders.h"

namespace tgr
{
    Point::Point() : Point(m3d::vec3()) {}

    Point::Point(const m3d::vec3& point) :
        Collider(ColliderType::Point), m_point(point) {}

    m3d::vec3 Point::getPoint()
    {
        return m_point;
    }

    Point& Point::setPoint(const m3d::vec3& point)
    {
        m_point = point;
        return *this;
    }

    Collision Point::checkCollision(const Collider* other) const
    {
        Collision res;

        return res;
    }

    m3d::vec3 Point::nearestPoint(const m3d::vec3& point) const
    {
        return m_point;
    }
}
