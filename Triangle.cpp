#include "TigerEngine/colliders.h"

#include <stdio.h>

namespace tgr
{
    Triangle::Triangle() : Triangle(m3d::vec3(), m3d::vec3(), m3d::vec3()) {}

    Triangle::Triangle(const m3d::vec3& p0, const m3d::vec3& p1, const m3d::vec3& p2) :
        Collider(ColliderType::Triangle)
        {
            m_points[0] = p0;
            m_points[1] = p1;
            m_points[2] = p2;
        }

    m3d::vec3* Triangle::getPoints()
    {
        return m_points;
    }

    m3d::vec3 Triangle::getPoint(const unsigned char& index)
    {
        return m_points[index];
    }

    Triangle& Triangle::setPoint(const unsigned char& index, const m3d::vec3& new_pos)
    {
        m_points[index] = new_pos;
        return *this;
    }

    Triangle& Triangle::setPoints(m3d::vec3 const * const points)
    {
        for(int i = 0; i < 3; i++)
            m_points[i] = points[i];
        return *this;
    }

    Triangle& Triangle::translate(const m3d::vec3& translation)
    {
        for(int i = 0; i < 3; i++)
            m_points[i] += translation;

        return *this;
    }

    Collision Triangle::checkCollision(const Collider* other) const
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

                res = b->checkCollision(this);
                res.normal *= -1.0f;

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

                m3d::vec3 p = b->getPoint();
                m3d::vec3 i = m_points[0];
                m3d::vec3 j = m_points[1];
                m3d::vec3 k = m_points[2];

                // Move the triangle so that the point becomes the
                // triangles origin
                i -= p;
                j -= p;
                k -= p;

                // Compute the normal vectors for triangles:
                // u = normal of PBC
                // v = normal of PCA
                // w = normal of PAB

                m3d::vec3 u = m3d::vec3::cross(j, k);
                m3d::vec3 v = m3d::vec3::cross(k, i);
                m3d::vec3 w = m3d::vec3::cross(i, j);

                // Test to see if the normals are facing
                // the same direction, return false if not
                if (m3d::vec3::dot(u, v) > 0.0f && m3d::vec3::dot(u, w) > 0.0f)
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

    m3d::vec3 Triangle::nearestPoint(const m3d::vec3& point) const
    {
        // convert to plane and find nearest point on plane
        Plane plane;
        plane.genFromPoints(m_points[0], m_points[1], m_points[2]);
        m3d::vec3 nearest_point = plane.nearestPoint(point);
        Point p = Point(nearest_point);
        // check if point is inside plane, if so, return that point
        if(checkCollision(&p).hit)
        {
            return nearest_point;
        }
        // find the nearest point on each of the edges

        Line edge1 = Line(m_points[0], m_points[1]);
        Line edge2 = Line(m_points[0], m_points[2]);
        Line edge3 = Line(m_points[1], m_points[2]);

        m3d::vec3 ep1 = edge1.nearestPoint(point);
        m3d::vec3 ep2 = edge2.nearestPoint(point);
        m3d::vec3 ep3 = edge3.nearestPoint(point);

        // return the point with the smallest distance from point
        float dist1 = m3d::vec3::lengthSqr(point - ep1);
        float dist2 = m3d::vec3::lengthSqr(point - ep2);
        float dist3 = m3d::vec3::lengthSqr(point - ep3);

        float min_dist = std::min(std::min(dist1, dist2), dist3);

        if(min_dist == dist1)
        {
            return ep1;
        }
        else if(min_dist == dist2)
        {
            return ep2;
        }
        else
        {
            return ep3;
        }
    }
}
