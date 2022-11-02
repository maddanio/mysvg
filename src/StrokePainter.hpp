#pragma once

#include "AK/Error.h"
#include "AK/StdLibExtras.h"
#include "Rasterizer.hpp"

#include "stdio.h"

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
        if (m_began)
            return AK::Error::from_string_literal("stroke already begun");
        m_thickness = thickness;
        m_closed = closed;
        m_first_point = m_current_point = p;
        m_began = true;
        return {};
    }
    AK::ErrorOr<void> stroke_to(point_t p, bool corner)
    {
        if (!m_began)
            return AK::Error::from_string_literal("stroke not begun");
        if (m_first)
        {
            if (m_closed)
            {
                m_second_point = p;
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
        if (m_closed)
        {
            TRY(stroke_to(m_first_point, false));
            TRY(stroke_to(m_second_point, true));
        }
        else
        {
            add_cap(m_last_point);
        }
        m_rasterizer.rasterize_edges(Rasterizer::FillRule::nonzero, paint);
        m_began = false;
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
        return (
            d2 * cross(p1, p1 + d1) - d1 * cross(p2, p2 + d2)
        ) / c;
    }
    static float norm(point_t p)
    {
        return AK::sqrt(p.x() * p.x() + p.y() * p.y());
    }
    static point_t normalized(point_t p)
    {
        return p / norm(p);
    }
    static float cross(point_t p1, point_t p2)
    {
        return p1.x() * p2.y() - p1.y() - p2.x();
    }
    static point_t left(point_t p)
    {
        return normalized(point_t{-p.y(), p.x()});
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
        printf("join\n");
        auto d1 = direction();
        auto d2 = direction(p);
        auto c = cross(d1, d2);
        if (abs(c) < 0.00001)
            return add_straight_join();
        auto o2 = offset(d2);
        auto l2 = m_current_point - o2;
        auto r2 = m_current_point + o2;
        // todo: reduce double calculations
        switch(m_join_type)
        {
        case JoinType::Bevel: {
            auto m = left(d1 - d2);
            if (c > 0)
            {
                auto left = intersect(m_left, d1, l2, d2, c);
                auto right1 = intersect(m_right, d1, m_current_point, m);
                auto right2 = intersect(r2, d2, m_current_point, m);
                add_edge({m_left, left});
                add_edge({right2, right1});
                add_edge({right1, m_right});
                m_left = left;
                m_right = right2;
            }
            else
            {
                auto right = intersect(m_right, d1, r2, d2, c);
                auto left1 = intersect(m_left, d1, m_current_point, m);
                auto left2 = intersect(l2, d2, m_current_point, m);
                add_edge({right, m_right});
                add_edge({m_left, left1});
                add_edge({left1, left2});
                m_left = left2;
                m_right = right;
            }
            break;
        }
        case JoinType::Miter: {
            auto left = intersect(m_left, d1, l2, d2, c);
            auto right = intersect(m_right, d1, r2, d2, c);
            add_edge({m_left, left});
            add_edge({right, m_right});
            m_left = left;
            m_right = right;
            break;
        }
        case JoinType::Round: {
            auto m = left(d1 - d2);
            if (c > 0)
            {
                auto left = intersect(m_left, d1, l2, d2, c);
                auto right1 = intersect(m_right, d1, m_current_point, m);
                auto right2 = intersect(r2, d2, m_current_point, m);
                add_edge({m_left, left});
                add_circle_segment(m_current_point, right1, d1, right2, d2);
                m_left = left;
                m_right = right2;
            }
            else
            {
                auto right = intersect(m_right, d1, r2, r2 + d2);
                auto left1 = intersect(m_left, d1, m_current_point, m);
                auto left2 = intersect(l2, l2 + d2, m_current_point, m);
                add_edge({right, m_right});
                add_circle_segment(m_current_point, left2, d2, left1, d1);
                m_right = right;
                m_left = left2;
            }
        }
        }
    }

    void add_straight_join()
    {
        auto d = direction();
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
            add_circle_segment(m_current_point, m_right, d, m_left, d);
            break;
        }
        }
    }
    void add_circle_segment(
        point_t c,
        point_t p1,
        point_t d1,
        point_t p2,
        point_t d2
    )
    {
        auto r1 = p1 - c;
        auto r = norm(r1);
        auto a1 = AK::atan2(-d2.x(), -d2.y());
        auto a2 = AK::atan2(d1.x(), d1.y());
        size_t n_steps = AK::max(1, norm(p2 - p1));
        point_t last = p1;
        for (size_t i = 0; i < n_steps; ++i)
        {
            auto a = (i + 1) * (a2 - a1) / (n_steps + 1) + a1;
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
        return m_current_point - m_last_point;
    }
    point_t direction(point_t next) const
    {
        return next - m_current_point;
    }
    Rasterizer m_rasterizer;
    float m_thickness = 1;
    bool m_closed;
    bool m_first = true;
    bool m_began = false;
    CapType m_cap_type = CapType::Butt;
    JoinType m_join_type = JoinType::Miter;
    point_t m_first_point;
    point_t m_second_point;
    point_t m_left;
    point_t m_right;
    point_t m_last_point;
    point_t m_current_point;
};
}
