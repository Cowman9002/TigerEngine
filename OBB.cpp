#include "TigerEngine/colliders.h"

#include <algorithm>

namespace tgr
{
    OBB::OBB() : OBB(m3d::vec3(), m3d::vec3(), m3d::quat()) {}

    OBB::OBB(m3d::vec3 center, m3d::vec3 extents, m3d::quat rotation) :
        Collider(ColliderType::OBB), m_center(center), m_extents(extents), m_rotation(rotation) {}

    m3d::vec3 OBB::getCenter() const
    {
        return m_center;
    }

    m3d::vec3 OBB::getExtents() const
    {
        return m_extents;
    }

    m3d::quat OBB::getRotation() const
    {
        return m_rotation;
    }

    OBB& OBB::setCenter(const m3d::vec3& center)
    {
        m_center = center;
        return *this;
    }

    OBB& OBB::setExtents(const m3d::vec3& extents)
    {
        m_extents = extents;
        return *this;
    }

    OBB& OBB::setRotation(const m3d::quat& rotation)
    {
        m_rotation = rotation;
        return *this;
    }

    OBB& OBB::translate(const m3d::vec3& translation)
    {
        m_center += translation;
        return *this;
    }

    Collision OBB::checkCollision(const Collider* other) const
    {
        return Collision{};
    }

    m3d::vec3 OBB::nearestPoint(const m3d::vec3& point) const
    {
        return m3d::vec3();
    }
}
