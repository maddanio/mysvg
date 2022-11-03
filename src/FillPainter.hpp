#pragma once

#include "Rasterizer.hpp"

namespace Gfx
{
class FillPainter
{
public:
    FillPainter(RefPtr<Gfx::Bitmap> image)
    : m_rasterizer{image}
    {
    }
    void begin(Rasterizer::point_t p)
    {
        m_first_point = m_current_point = p;
    }
    void edge_to(Rasterizer::point_t p)
    {
        m_rasterizer.add_edge({m_current_point, p});
        m_current_point = p;
    }
    void end()
    {
        m_rasterizer.add_edge({m_current_point, m_first_point});

    }
    void end_shape(Rasterizer::Paint const& paint)
    {
        m_rasterizer.rasterize_edges(Rasterizer::FillRule::evenodd, paint);
    }
private:
    Rasterizer m_rasterizer;
    Rasterizer::point_t m_first_point;
    Rasterizer::point_t m_current_point;
};
}
