#pragma once

#include "Collider.h"

#include <vector>

namespace tgr
{
    class AABB : public Collider
    {
    private:
        m3d::vec3 max_bounds;
        m3d::vec3 min_bounds;

    public:
        AABB();
        AABB(const m3d::vec3& max_bounds, const m3d::vec3& min_bounds);

        AABB& genFromPoints(const m3d::vec3 *points, unsigned long int num_points);
        AABB& genFromPoints(const std::vector<m3d::vec3>& points);

        m3d::vec3 getCenter() const;
        m3d::vec3 getExtents() const;
        m3d::vec3 getMaxBounds() const;
        m3d::vec3 getMinBounds() const;

        AABB& setMaxBounds(const m3d::vec3& new_max);
        AABB& setMinBounds(const m3d::vec3& new_min);
        AABB& setCenterExtents(const m3d::vec3& center, const m3d::vec3& extents);

        // sets min and max bounds to min and max values of each
        AABB& normalize();
        AABB& translate(const m3d::vec3& translation);

        virtual Collision checkCollision(const Collider* other) const override;
        virtual m3d::vec3 nearestPoint(const m3d::vec3& point) const override;
    };
}
