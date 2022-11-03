#pragma once

#include "AK/Error.h"
#include "AK/RefPtr.h"
#include "AK/StdLibExtras.h"
#include "FillPainter.hpp"
#include "Rasterizer.hpp"
#include "StrokePainter.hpp"

namespace Gfx
{
class PathPainter
{
public:
    enum class StrokeKind {OpenStroke, ClosedStroke, NoStroke};
    enum class FillKind {Filled, NotFilled};
    using point_t = Rasterizer::point_t;
    PathPainter(RefPtr<Gfx::Bitmap> image)
    : m_stroke_painter{image}
    , m_fill_painter{image}
    {
    }

    AK::ErrorOr<void> begin(point_t p, StrokeKind stroke_kind, FillKind fill_kind, float thickness = 0)
    {
        printf("-begin\n");
        m_stroke_kind = stroke_kind;
        m_fill_kind = fill_kind;
        m_thickness = thickness;
        TRY(begin_paint(p));
        m_position = p;
        return {};
    }

    AK::ErrorOr<void> cubic_bezier_to(
        point_t p2,
        point_t p3,
        point_t p4
    )
    {
        printf("-bezier\n");
        TRY(flatten_cubic_bezier(m_position, p2, p3, p4));
        m_position = p4;
        return {};
    }

    AK::ErrorOr<void> quadratic_bezier_to(
        point_t p2,
        point_t p3
    )
    {
        TRY(flatten_quadratic_bezier(m_position, p2, p3));
        m_position = p3;
        return {};
    }

    AK::ErrorOr<void> line_to(
        point_t p2
    )
    {
        TRY(edge_to(p2, true));
        m_position = p2;
        return {};
    }

    AK::ErrorOr<void> end(Rasterizer::Paint const& stroke_paint, Rasterizer::Paint const& fill_paint)
    {
        printf("-end\n");
        if (m_fill_kind == FillKind::Filled)
            m_fill_painter.end(fill_paint);
        switch(m_stroke_kind)
        {
        case StrokeKind::OpenStroke:
        case StrokeKind::ClosedStroke:
            TRY(m_stroke_painter.end(stroke_paint));
            break;
        case StrokeKind::NoStroke:
            break;
        }
        return {};
    }

private:

    enum class Phase {Idle, Begun, Painting};

    AK::ErrorOr<void> flatten_cubic_bezier(
        point_t p1,
        point_t p2,
        point_t p3,
        point_t p4,
        int level = 0,
        bool corner = true
    )
    {
        auto good_enough = [&]{
            constexpr float tessTol = 0.25f;
            auto d = p4 - p1;
            if (AK::max(AK::abs(d.x()), AK::abs(d.y())) < tessTol)
                return true;
            auto a = AK::abs(cross(p2 - p1, d)) + AK::abs(cross(p3 - p4, d));
            return a * a < tessTol * dot(d, d);
        };
        if (level > 10 || good_enough()) {
            TRY(edge_to(p4, corner));
        } else {
            auto p12 = (p1 + p2) / 2;
            auto p23 = (p2 + p3) / 2;
            auto p34 = (p3 + p4) / 2;
            auto p123 = (p12 + p23) / 2;
            auto p234 = (p23 + p34) / 2;
            auto p1234 = (p123 + p234) / 2;
            TRY(flatten_cubic_bezier(p1, p12, p123, p1234, level+1, true));
            TRY(flatten_cubic_bezier(p1234, p234, p34, p4, level+1, false));
        }
        return {};
    }

    AK::ErrorOr<void> flatten_quadratic_bezier(
        point_t p1,
        point_t p2,
        point_t p3,
        int level = 0,
        bool corner = true
    )
    {
        auto good_enough = [&]{
            constexpr float tessTol = 0.25f;
            auto d = p3 - p1;
            if (AK::max(AK::abs(d.x()), AK::abs(d.y())) < tessTol)
                return true;
            auto a = AK::abs(cross(p2 - p1, d));
            return a * a < tessTol * dot(d, d);
        };
        if (level > 10 || good_enough()) {
            TRY(edge_to(p3, corner));
        } else {
            auto p12 = (p1 + p2) / 2;
            auto p23 = (p2 + p3) / 2;
            auto p123 = (p12 + p23) / 2;
            TRY(flatten_quadratic_bezier(p1, p12, p123, level+1, true));
            TRY(flatten_quadratic_bezier(p123, p23, p3, level+1, false));
        }
        return {};
    }

    static float cross(point_t p1, point_t p2)
    {
        return p1.x() * p2.y() - p1.y() * p2.x();
    }

    static float dot(point_t p1, point_t p2)
    {
        return p1.x() * p2.x() + p1.y() * p2.y();
    }

    AK::ErrorOr<void> begin_paint(point_t p)
    {
        switch(m_stroke_kind)
        {
        case StrokeKind::OpenStroke:
            TRY(m_stroke_painter.begin(p, false, m_thickness));
            break;
        case StrokeKind::ClosedStroke:
            TRY(m_stroke_painter.begin(p, true, m_thickness));
            break;
        case StrokeKind::NoStroke:
            break;
        }
        if (m_fill_kind == FillKind::Filled)
            m_fill_painter.begin(p);
        return {};
    }

    AK::ErrorOr<void> edge_to(point_t p, bool corner)
    {
        switch(m_stroke_kind)
        {
        case StrokeKind::OpenStroke:
        case StrokeKind::ClosedStroke:
            TRY(m_stroke_painter.stroke_to(p, corner));
            break;
        case StrokeKind::NoStroke:
            break;
        }
        if (m_fill_kind == FillKind::Filled)
            m_fill_painter.edge_to(p);
        return {};
    }

    StrokePainter m_stroke_painter;
    FillPainter m_fill_painter;
    StrokeKind m_stroke_kind;
    FillKind m_fill_kind;
    float m_thickness;
    point_t m_position;
};
}
