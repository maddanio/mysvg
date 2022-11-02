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

    AK::ErrorOr<void> begin(StrokeKind stroke_kind, FillKind fill_kind, float thickness = 0)
    {
        if (m_phase != Phase::Idle)
            return AK::Error::from_string_literal("PathPainter not idle when beginning");
        m_stroke_kind = stroke_kind;
        m_fill_kind = fill_kind;
        m_thickness = thickness;
        m_phase = Phase::Begun;
        return {};
    }

    AK::ErrorOr<void> cubic_bezier(
        point_t p1,
        point_t p2,
        point_t p3,
        point_t p4
    )
    {
        TRY(emit(p1, true));
        TRY(flatten_cubic_bezier(p1, p2, p3, p4, 0));
        return {};
    }

    AK::ErrorOr<void> end(Rasterizer::Paint const& stroke_paint, Rasterizer::Paint const& fill_paint)
    {
        if (m_phase == Phase::Idle)
        {
            return AK::Error::from_string_literal("PathPainter idle when ending");
        }
        if (m_phase == Phase::Painting)
        {
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
        }
        m_phase = Phase::Idle;
        return {};
    }

private:

    enum class Phase {Idle, Begun, Painting};

    AK::ErrorOr<void> flatten_cubic_bezier(
        point_t p1,
        point_t p2,
        point_t p3,
        point_t p4,
        int level
    )
    {
        constexpr float tessTol = 0.25f;
        if (level > 10) return {};
        auto cross = [](point_t a, point_t b){return a.x() * b.y() - a.y() * b.x();};
        auto dot = [](point_t a, point_t b){return a.x() * b.x() + a.y() * b.y();};
        auto d = p4 - p1;
        auto a = AK::abs(cross(p2 - p4, d)) + AK::abs(cross(p3 - p4, d));
        if (a * a < tessTol * dot(d, d)) {
            TRY(edge_to({p4.x(), p4.y()}, false));
        }
        auto p12 = (p1 + p2) / 2;
        auto p23 = (p2 + p3) / 2;
        auto p34 = (p3 + p4) / 2;
        auto p123 = (p12 + p23) / 2;
        auto p234 = (p23 + p34) / 2;
        auto p1234 = (p123 + p234) / 2;
        TRY(flatten_cubic_bezier(p1, p12, p123, p1234, level+1));
        TRY(flatten_cubic_bezier(p1234, p234, p34, p4, level+1));
        return {};
    }

    AK::ErrorOr<void> emit(point_t p, bool corner)
    {
        switch(m_phase)
        {
        case Phase::Idle:
            return AK::Error::from_string_literal("PathPainter idle when painting");
        case Phase::Begun:
            TRY(begin_paint(p));
            m_phase = Phase::Painting;
            break;
        case Phase::Painting:
            TRY(edge_to(p, corner));
            break;
        }
        return {};
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

    Phase m_phase = Phase::Idle;
    StrokePainter m_stroke_painter;
    FillPainter m_fill_painter;
    StrokeKind m_stroke_kind;
    FillKind m_fill_kind;
    float m_thickness;
};
}
