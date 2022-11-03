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

namespace Gfx {

class Rasterizer
{
public:
    using point_t = Point<float>;
    using color_t = Color;
    using image_t = RefPtr<Bitmap>;
    struct Paint
    {
        color_t color;
    };
    struct Edge
    {
        point_t from, to;
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
    void fill_scanline(size_t i, const Paint& paint);
    static constexpr uint8_t _oversampling = 5;
    size_t _min_col, _max_col;
    image_t _image;
    Painter _painter;
    AK::Vector<uint8_t> _coverage;
    AK::Vector<Edge> _edges;
    AK::Vector<ActiveEdge> _active_edges;
};
}
