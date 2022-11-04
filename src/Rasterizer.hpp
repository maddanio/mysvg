#pragma once

#include <AK/RefPtr.h>
#include <AK/StdLibExtras.h>
#include <LibGfx/Color.h>
#include <AK/Math.h>
#include <AK/Vector.h>
#include <AK/RefCounted.h>
#include <LibGfx/Point.h>
#include <LibGfx/Bitmap.h>
#include <LibGfx/Painter.h>

#include "AK/Variant.h"
#include "PointUtils.hpp"
#include "Gradients.hpp"

namespace Gfx {

class Rasterizer
{
public:
    using image_t = RefPtr<Bitmap>;
    struct Paint
    {
        AK::Variant<Color, LinearGradient> coloring;
    };
    struct Edge
    {
        Point<float> from, to;
        float top() const {return AK::min(from.y(), to.y());}
        float bottom() const {return AK::max(from.y(), to.y());}
    };
    enum class FillRule {nonzero, evenodd};
    Rasterizer(image_t image)
    : _image{image}
    , _painter{*_image}
    {
        _coverage.resize(_image->width());
    }
    void add_edge(Edge edge)
    {
        if (edge.from.y() != edge.to.y())
            _edges.append(edge);
    }
    void rasterize_edges(FillRule fill_rule, const Paint& paint);
private:
    struct ActiveEdge
    {
        float x, dx, end;
        int winding;
        ActiveEdge(const Edge& edge, float y);
    };
    void rasterize_scanline(FillRule fill_rule);
    void update_coverage(float x0, float x1);
    void fill_scanline_solid(size_t i, Color in_color);
    void fill_scanline_linear_gradient(size_t i, const LinearGradient& gradient);
    static constexpr uint8_t _oversampling = 5;
    size_t _min_col, _max_col;
    image_t _image;
    Painter _painter;
    AK::Vector<uint8_t> _coverage;
    AK::Vector<Edge> _edges;
    AK::Vector<ActiveEdge> _active_edges;
};
}
