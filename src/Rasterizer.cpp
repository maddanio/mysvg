#include "Rasterizer.hpp"
#include "LibGfx/Color.h"

#include <AK/QuickSort.h>

namespace Gfx {

void Rasterizer::rasterize_edges(FillRule fill_rule, const Paint& paint)
{
    AK::quick_sort(_edges, [](auto a, auto b){return a.top() < b.top();});
    auto next_edge = _edges.begin();
    // todo: use min-heap?
    for (int i = 0; i < _image->height(); ++i)
    {
        _active_edges.remove_all_matching([i](auto& e){return e.end <= float(i);});
        for (auto& edge : _active_edges)
            edge.x += edge.dx;
        // todo: use pivot element at the end
        for(;next_edge != _edges.end() && next_edge->top() <= float(i);++next_edge)
        {
            if (next_edge->bottom() > float(i))
                _active_edges.append(ActiveEdge{*next_edge});
        }
        AK::quick_sort(_active_edges, [](auto a, auto b){return a.x < b.x;});
        rasterize_scanline(i, fill_rule, paint);
    }
    _edges.clear();
    _active_edges.clear();
}

Rasterizer::ActiveEdge::ActiveEdge(const Rasterizer::Edge& edge)
{
    auto from = edge.from;
    auto to = edge.to;
    if (from.y() > to.y())
    {
        swap(from, to);
        winding = -1;
    }
    else
    {
        winding = 1;
    }
    auto d = to - from;
    dx = d.x() / d.y();
    x = from.x();
    end = to.y();
}

void Rasterizer::rasterize_scanline(size_t i, FillRule fill_rule, const Paint& paint)
{
    float x = 0;
    int winding = 0;
    switch(fill_rule)
    {
        case FillRule::nonzero:
            for (auto& edge : _active_edges)
            {
                if (winding == 0)
                {
                    x = edge.x;
                    winding = edge.winding;
                }
                else
                {
                    winding += edge.winding;
                    if (winding == 0)
                        fill_scanline(i, x, edge.x, paint);
                }
            }
        break;
        case FillRule::evenodd:
            for (auto& edge : _active_edges)
            {
                if (winding == 0)
                {
                    x = edge.x;
                    winding = 1;
                }
                else
                {
                    winding = 0;
                    fill_scanline(i, x, edge.x, paint);
                }
            }
        break;
    }
}

void Rasterizer::fill_scanline(size_t i, float x0, float x1, const Paint& paint)
{
    auto row = _image->scanline(i);
    size_t first_index = x0;
    size_t last_index = x1;
    auto start_coverage = first_index + 1 - x0;
    auto end_coverage = x1 - last_index;
    auto put = [row](int i, Gfx::Color color){
        row[i] = Color::from_argb(row[i]).blend(color).value();
    };
    if (first_index == last_index)
    {
        put(first_index, paint.color.with_alpha(paint.color.alpha() * start_coverage * end_coverage));
    }
    else
    {
        put(first_index, paint.color.with_alpha(paint.color.alpha() * start_coverage));
        put(last_index, paint.color.with_alpha(paint.color.alpha() * end_coverage));
        for (size_t i = first_index + 1; i <= last_index - 1; ++i)
            put(i, paint.color);
    }
}

}
