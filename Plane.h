#pragma once

#include "Collider.h"

namespace tgr
{
    class Plane : public Collider
    {
    private:
    public:
        Plane();

        virtual Collision checkCollision(const Collider* other) const override;
        virtual m3d::vec3 nearestPoint(const m3d::vec3& point) const override;
    };
}
