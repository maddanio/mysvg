#pragma once

#include "AK/StdLibExtras.h"
#include "Rasterizer.hpp"

namespace Gfx
{
class StrokePainter
{
public:
    enum class LineEnd {Start, End};
    enum class CapType {Butt, Square, Round};
    enum class JoinType {Bevel, Miter, Round, Straight};
    StrokePainter(Rasterizer::image_t image)
    : m_rasterizer{image}
    {
    }
    void begin(Rasterizer::point_t p, float thickness)
    {
        m_thickness = thickness;
        m_first_point = m_current_point = p;
    }
    void stroke_to(Rasterizer::point_t p, bool corner)
    {
        if (m_first)
            add_cap(LineEnd::Start);
        else
            add_join(p, corner);
        m_last_point = m_current_point;
        m_current_point = p;
        m_first = false;
    }
    void end()
    {
        add_cap(LineEnd::End);
    }
    void close()
    {
        add_join(m_first_point, false);
    }
private:
    static Rasterizer::point_t intersect(
        Rasterizer::point_t p1,
        Rasterizer::point_t p2,
        Rasterizer::point_t p3,
        Rasterizer::point_t p4
    )
    {
        auto den = cross(p1 - p2, p3 - p4);
        if (abs(den) < 0.00001f)
            return p2;
        return (
            (p3 - p4) * cross(p1, p2) - (p1 - p2) * cross(p3, p4)
        ) / den;
    }
    static float norm(Rasterizer::point_t p)
    {
        return AK::sqrt(p.x() * p.x() + p.y() * p.y());
    }
    static Rasterizer::point_t normalized(Rasterizer::point_t p)
    {
        return p / norm(p);
    }
    static float cross(Rasterizer::point_t p1, Rasterizer::point_t p2)
    {
        return p1.x() * p2.y() - p1.y() - p2.x();
    }
    static Rasterizer::point_t orthogonal(Rasterizer::point_t p)
    {
        return {p.y(), -p.x()};
    }
    void add_edge(Rasterizer::Edge edge)
    {
        m_rasterizer.add_edge(edge);
    }
    void add_join(Rasterizer::point_t p, bool corner)
    {
        // todo: reduce double calculations
        switch(corner ? m_join_type : JoinType::Straight)
        {
        case JoinType::Straight: {
            auto d = direction();
            auto o = normalized(orthogonal(d)) * (m_thickness / 2);
            auto left = m_last_point + d - o;
            auto right = m_last_point + d + o;
            add_edge({m_left, left});
            add_edge({m_right, right});
            m_left = left;
            m_right = right;
            break;
        }
        case JoinType::Bevel: {
            auto d1 = direction();
            auto d2 = direction(p);
            auto c = cross(d1, d2);
            if (abs(c) < 0.00001)
                return add_join(p, false);
            auto o2 = normalized(orthogonal(d2)) * (m_thickness / 2);
            auto m = normalized(orthogonal(d1 - d2));
            if (c > 0)
            {
                auto left = intersect(m_left, m_left + d1, m_current_point - o2, m_current_point - o2 + d2);
                auto right1 = intersect(m_right, m_right + d1, m_current_point, m_current_point - m);
                auto right2 = intersect(m_current_point + o2, m_current_point + o2 + d2, m_current_point, m_current_point + m);
                add_edge({m_left, left});
                add_edge({right2, right1});
                add_edge({right1, m_right});
                m_left = left;
                m_right = right2;
            }
            else
            {
                auto right = intersect(m_right, m_right + d1, m_current_point + o2, m_current_point + o2 + d2);
                auto left1 = intersect(m_left, m_left + d1, m_current_point, m_current_point - m);
                auto left2 = intersect(m_current_point - o2, m_current_point - o2 + d2, m_current_point, m_current_point + m);
                add_edge({right, m_right});
                add_edge({left2, left1});
                add_edge({left1, m_left});
                m_right = right;
                m_left = left2;
            }
            break;
        }
        case JoinType::Miter: {
            auto d1 = direction();
            auto d2 = direction(p);
            auto c = cross(d1, d2);
            if (abs(c) < 0.00001)
                return add_join(p, false);
            auto o2 = normalized(orthogonal(d2)) * (m_thickness / 2);
            auto left = intersect(m_left, m_left + d1, m_current_point - o2, m_current_point - o2 + d2);
            auto right = intersect(m_right, m_right + d1, m_current_point + o2, m_current_point + o2 + d2);
            add_edge({m_left, left});
            add_edge({right, m_right});
            m_left = left;
            m_right = right;
            break;
        }
        case JoinType::Round: {
            break;
            auto d1 = direction();
            auto d2 = direction(p);
            auto c = cross(d1, d2);
            if (abs(c) < 0.00001)
                return add_join(p, false);
            auto o2 = normalized(orthogonal(d2)) * (m_thickness / 2);
            auto m = normalized(orthogonal(d1 - d2));
            if (c > 0)
            {
                auto left = intersect(m_left, m_left + d1, m_current_point - o2, m_current_point - o2 + d2);
                auto right1 = intersect(m_right, m_right + d1, m_current_point, m_current_point - m);
                auto right2 = intersect(m_current_point + o2, m_current_point + o2 + d2, m_current_point, m_current_point + m);
                add_edge({m_left, left});
                add_circle_segment(m_current_point, right1, d1, right2, d2);
                m_left = left;
                m_right = right2;
            }
            else
            {
                auto right = intersect(m_right, m_right + d1, m_current_point + o2, m_current_point + o2 + d2);
                auto left1 = intersect(m_left, m_left + d1, m_current_point, m_current_point - m);
                auto left2 = intersect(m_current_point - o2, m_current_point - o2 + d2, m_current_point, m_current_point + m);
                add_edge({right, m_right});
                add_circle_segment(m_current_point, left2, d2, left1, d1);
                m_right = right;
                m_left = left2;
            }
        }
        }
    }
    void add_cap(LineEnd end)
    {
        auto d = direction();
        if (end == LineEnd::End)
            d = d * -1;
        auto offset = orthogonal(d) * m_thickness;
        m_left = m_current_point + offset;
        m_right = m_current_point - offset;
        switch(m_cap_type)
        {
        case CapType::Butt:
        {
            add_edge({m_left, m_right});
            break;
        }
        case CapType::Square:
        {
            m_left -= d / 2;
            m_right -= d / 2;
            add_edge({m_left, m_right});
            break;
        }
        case CapType::Round:
        {
            add_circle_segment(m_current_point, m_left, d, m_right, d);
            break;
        }
        }
    }
    void add_circle_segment(
        Rasterizer::point_t c,
        Rasterizer::point_t p1,
        Rasterizer::point_t d1,
        Rasterizer::point_t p2,
        Rasterizer::point_t d2
    )
    {
        auto r1 = p1 - c;
        auto r = norm(r1);
        auto a1 = AK::atan2(d2.x(), d2.y());
        auto a2 = AK::atan2(d1.x(), d1.y());
        size_t n_steps = AK::max(1, norm(p2 - p1));
        Rasterizer::point_t last = p1;
        for (size_t i = 0; i < n_steps; ++i)
        {
            auto a = (i + 1) * (a2 - a1) / (n_steps + 1) + a1;
            Rasterizer::point_t p{AK::sin(a) * r, AK::cos(a) * r};
            add_edge({last, p});
            last = p;
        }
        add_edge({last, p2});
    }
    Rasterizer::point_t direction() const
    {
        return m_current_point - m_last_point;
    }
    Rasterizer::point_t direction(Rasterizer::point_t next) const
    {
        return next - m_current_point;
    }
    Rasterizer m_rasterizer;
    float m_thickness = 1;
    bool m_first = true;
    CapType m_cap_type;
    JoinType m_join_type;
    Rasterizer::point_t m_first_point;
    Rasterizer::point_t m_left;
    Rasterizer::point_t m_right;
    Rasterizer::point_t m_last_point;
    Rasterizer::point_t m_current_point;
};
}
