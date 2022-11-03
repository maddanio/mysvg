#pragma once

#include "AK/Error.h"
#include "AK/StdLibExtras.h"
#include "Rasterizer.hpp"

#include "stdio.h"
#include <cmath>

namespace Gfx
{
class StrokePainter
{
public:
    enum class LineEnd {Start, End};
    enum class CapType {Butt, Square, Round};
    enum class JoinType {Bevel, Miter, Round};
    enum class EndType {CloseWithCorner, CloseWihoutCorner, Open};
    using point_t = Rasterizer::point_t;
    StrokePainter(Rasterizer::image_t image)
    : m_rasterizer{image}
    {
    }
    AK::ErrorOr<void> begin(point_t p, bool closed, float thickness)
    {
        printf(">begin %f,%f,%s\n",p.x(), p.y(), closed?"closed":"");
        m_thickness = thickness;
        m_closed = closed;
        m_first_point = m_current_point = p;
        m_first = true;
        return {};
    }
    AK::ErrorOr<void> stroke_to(point_t p, bool corner)
    {
        if (p.distance_from(m_current_point) < 0.1f)
            return {};
        printf(">%f,%f,%s\n",p.x(), p.y(), corner?"corner":"");
        if (m_first)
        {
            m_second_point = p;
            if (m_closed)
            {
                auto o = offset(direction(p));
                m_left = m_current_point - o;
                m_right = m_current_point + o;
            }
            else
            {
                add_cap(p);
            }
        }
        else if (corner)
        {
            add_join(p);
        }
        else
        {
            add_straight_join();
        }
        m_last_point = m_current_point;
        m_current_point = p;
        m_first = false;
        return {};
    }
    AK::ErrorOr<void> end(Rasterizer::Paint const& paint)
    {
        printf("<end\n");
        if (m_first)
        {
            printf(">end\n");
            return {};
        }
        if (m_closed)
        {
            if (m_current_point.distance_from(m_first_point) > 0.1f)
                TRY(stroke_to(m_first_point, true));
            TRY(stroke_to(m_second_point, true));
        }
        else
        {
            add_cap(m_last_point);
        }
        m_rasterizer.rasterize_edges(Rasterizer::FillRule::nonzero, paint);
        printf(">end\n");
        return {};
    }
private:
    static point_t intersect(
        point_t p1,
        point_t d1,
        point_t p2,
        point_t d2
    )
    {
        return intersect(p1, d1, p2, d2, cross(d1, d2));
    }
    static point_t intersect(
        point_t p1,
        point_t d1,
        point_t p2,
        point_t d2,
        float c
    )
    {
        return -(
            d2 * cross(p1, p1 + d1) - d1 * cross(p2, p2 + d2)
        ) / c;
    }
    static float norm(point_t p)
    {
        auto result = AK::sqrt(p.x() * p.x() + p.y() * p.y());
        if (result < 0.0001f)
            printf("norm %f\n", result);
        return result;
    }
    static point_t normalized(point_t p)
    {
        return p / norm(p);
    }
    static float cross(point_t p1, point_t p2)
    {
        return p1.x() * p2.y() - p1.y() * p2.x();
    }
    static point_t left(point_t p)
    {
        return point_t{-p.y(), p.x()};
    }
    point_t offset(point_t d) const
    {
        return left(d) * (m_thickness / 2);
    }
    void add_edge(Rasterizer::Edge edge)
    {
        m_rasterizer.add_edge(edge);
    }
    void add_join(point_t p)
    {
        auto d1 = direction();
        auto d2 = direction(p);
        auto o1 = offset(d1);
        auto o2 = offset(d2);
        auto l1 = m_current_point - o1;
        auto r1 = m_current_point + o1;
        auto l2 = m_current_point - o2;
        auto r2 = m_current_point + o2;
        switch(m_join_type)
        {
        case JoinType::Bevel: {
            add_edge({m_left, l1});
            add_edge({l1, l2});
            add_edge({r2, r1});
            add_edge({r1, m_right});
            m_left = l2;
            m_right = r2;
            break;
        }
        case JoinType::Miter: {
            auto c = cross(d1, d2);
            if (c > 0)
            {
                auto left = intersect(m_left, d1, l2, d2, c);
                printf("left %f,%f\n", left.x(), left.y());
                add_edge({m_left, left});
                add_edge({r2, r1});
                add_edge({r1, m_right});
                m_left = left;
                m_right = r2;
            }
            else
            {
                auto right = intersect(m_right, d1, r2, d2, c);
                printf("right %f,%f\n", right.x(), right.y());
                add_edge({right, m_right});
                add_edge({m_left, l1});
                add_edge({l1, l2});
                m_left = l2;
                m_right = right;
            }
            break;
        }
        case JoinType::Round: {
            auto c = cross(d1, d2);
            if (c > 0)
            {
                add_edge({m_left, l1});
                add_circle_segment(m_current_point, l1, l2);
                add_edge({r2, r1});
                add_edge({r1, m_right});
                m_left = l2;
                m_right = r2;
            }
            else
            {
                add_edge({m_left, l1});
                add_edge({l1, l2});
                add_circle_segment(m_current_point, r2, r1);
                add_edge({r1, m_right});
                m_left = l2;
                m_right = r2;
            }
        }
        }
    }

    void add_straight_join()
    {
        auto d = direction();
        if (AK::max(AK::abs(d.x()), AK::abs(d.y())) < 0.01f)
            return;
        auto o = offset(d);
        auto left = m_current_point - o;
        auto right = m_current_point + o;
        add_edge({m_left, left});
        add_edge({right, m_right});
        m_left = left;
        m_right = right;
    }


    void add_cap(point_t towards)
    {
        printf("cap\n");
        auto d = direction(towards);
        auto o = offset(d);
        m_left = m_current_point - o;
        m_right = m_current_point + o;
        switch(m_cap_type)
        {
        case CapType::Butt:
        {
            add_edge({m_right, m_left});
            break;
        }
        case CapType::Square:
        {
            auto backoff = normalized(d) * (m_thickness / 2);
            m_left -= backoff;
            m_right -= backoff;
            add_edge({m_right, m_left});
            break;
        }
        case CapType::Round:
        {
            add_circle_segment(m_current_point, m_right, m_left);
            break;
        }
        }
    }

    void add_circle_segment(
        point_t c,
        point_t p1,
        point_t p2
    )
    {
        auto r = p1.distance_from(c);
        auto r1 = p1 - c;
        auto r2 = p2 - c;
        auto a1 = AK::atan2(r1.x(), r1.y());
        auto a2 = AK::atan2(r2.x(), r2.y());
        if (cross(r1, r2) < 0)
        {
            if (a1 > a2)
                a1 -= 2 * M_PI;
        }
        else
        {
            if (a1 < a2)
                a2 -= 2 * M_PI;
        }
        size_t n_steps = AK::max(1, p1.distance_from(p2));
        point_t last = p1;
        for (size_t i = 0; i < n_steps; ++i)
        {
            auto a = a1 + float(i + 1) / float(n_steps + 1) * (a2 - a1);
            float s, c;
            AK::sincos(a, s, c);
            auto p = m_current_point + point_t{s, c} * r;
            add_edge({last, p});
            last = p;
        }
        add_edge({last, p2});
    }

    point_t direction() const
    {
        return normalized(m_current_point - m_last_point);
    }

    point_t direction(point_t next) const
    {
        return normalized(next - m_current_point);
    }

    Rasterizer m_rasterizer;
    float m_thickness = 1;
    bool m_closed;
    bool m_first = true;
    CapType m_cap_type = CapType::Butt;
    JoinType m_join_type = JoinType::Round;
    point_t m_first_point;
    point_t m_second_point;
    point_t m_left;
    point_t m_right;
    point_t m_last_point;
    point_t m_current_point;
};
}
