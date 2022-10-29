#pragma once

#include "AK/RefPtr.h"
#include "FillPainter.hpp"
#include "StrokePainter.hpp"

namespace Gfx
{
class PathPainter
{
public:
    enum class StrokeKind {OpenStroke, ClosedStroke, NoStroke};
    enum class FillKind {Filled, NotFilled};
    PathPainter(RefPtr<Gfx::Bitmap> image)
    : m_stroke_painter{image}
    , m_fill_painter{image}
    {
    }
    void begin(Rasterizer::point_t p, StrokeKind stroke_kind, FillKind fill_kind, float thickness = 0)
    {
        m_stroke_kind = stroke_kind;
        m_fill_kind = fill_kind;
        switch(stroke_kind)
        {
        case StrokeKind::OpenStroke:
            m_stroke_painter.begin(p, false, thickness);
            break;
        case StrokeKind::ClosedStroke:
            m_stroke_painter.begin(p, true, thickness);
            break;
        case StrokeKind::NoStroke:
            break;
        }
        if (fill_kind == FillKind::Filled)
            m_fill_painter.begin(p);
    }
    void edge_to(Rasterizer::point_t p, bool corner)
    {
        switch(m_stroke_kind)
        {
        case StrokeKind::OpenStroke:
        case StrokeKind::ClosedStroke:
            m_stroke_painter.stroke_to(p, corner);
            break;
        case StrokeKind::NoStroke:
            break;
        }
        if (m_fill_kind == FillKind::Filled)
            m_fill_painter.begin(p);
    }
    void end(Rasterizer::Paint const& stroke_paint, Rasterizer::Paint const& fill_paint, bool corner)
    {
        switch(m_stroke_kind)
        {
        case StrokeKind::OpenStroke:
        case StrokeKind::ClosedStroke:
            m_stroke_painter.end(stroke_paint, corner);
            break;
        case StrokeKind::NoStroke:
            break;
        }
        if (m_fill_kind == FillKind::Filled)
            m_fill_painter.end(fill_paint);
    }
private:
    StrokePainter m_stroke_painter;
    FillPainter m_fill_painter;
    StrokeKind m_stroke_kind;
    FillKind m_fill_kind;
};
}
