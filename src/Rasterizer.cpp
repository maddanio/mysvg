#include "Rasterizer.hpp"
#include "LibGfx/Color.h"

#include <AK/QuickSort.h>

namespace Gfx {

void Rasterizer::rasterize_edges(FillRule fill_rule, const Paint& paint)
{
    #if 0
    for (auto& edge : _edges)
        _painter.draw_line(Point<int>(edge.from), Point<int>(edge.to), Color::White);
    return;
    #endif
    AK::quick_sort(_edges, [](auto a, auto b){return a.top() < b.top();});
    auto next_edge = _edges.begin();
    // todo: use min-heap?
    for (int i = 0; i < _image->height(); ++i)
    {
        _min_col = _image->width() - 1;
        _max_col = 0;
        memset(_coverage.data(), 0, _coverage.size());
        for (size_t s = 0; s < _oversampling; ++s)
        {
            auto scany = i + float(s) / _oversampling;
            _active_edges.remove_all_matching([scany](auto& e){return e.end <= scany;});
            for (auto& edge : _active_edges)
                edge.x += edge.dx / _oversampling;
            for(;next_edge != _edges.end() && next_edge->top() <= scany;++next_edge)
            {
                if (next_edge->bottom() > scany)
                    _active_edges.append(ActiveEdge{*next_edge, scany});
            }
            AK::quick_sort(_active_edges, [](auto a, auto b){return a.x < b.x;});
            rasterize_scanline(fill_rule);
        }
        if (_min_col <= _max_col)
            fill_scanline(i, paint);
    }
    _edges.clear();
    _active_edges.clear();
}

Rasterizer::ActiveEdge::ActiveEdge(const Rasterizer::Edge& edge, float y)
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
    x = from.x() + dx * (y - from.y());
    end = to.y();
}

void Rasterizer::rasterize_scanline(FillRule fill_rule)
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
                        update_coverage(x, edge.x);
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
                    update_coverage(x, edge.x);
                }
            }
        break;
    }
}

void Rasterizer::update_coverage(float x0, float x1)
{
    size_t first_index = AK::clamp<size_t>(x0, 0, _image->width() - 1);
    size_t last_index = AK::clamp<size_t>(x1, 0, _image->width() - 1);
    _min_col = AK::min(_min_col, first_index);
    _max_col = AK::max(_max_col, last_index);
    auto start_coverage = AK::clamp(first_index + 1 - x0, 0, 1);
    auto end_coverage = AK::clamp(x1 - last_index, 0, 1);
    if (first_index == last_index)
    {
        _coverage[first_index] += (AK::min(start_coverage * end_coverage, 1) * 255) / _oversampling;
    }
    else
    {
        _coverage[first_index] += (AK::min(start_coverage, 1) * 255) / _oversampling;
        for (size_t i = first_index + 1; i < last_index; ++i)
        {
            _coverage[i] += 255 / _oversampling;
        }
        _coverage[last_index] += (AK::min(end_coverage, 1) * 255) / _oversampling;
    }
}

void Rasterizer::fill_scanline(size_t i, const Paint& paint)
{
    auto row = _image->scanline(i);
    for (size_t j = _min_col; j <= _max_col; ++j)
    {
        auto color = paint.color.with_alpha((uint16_t(paint.color.alpha()) * _coverage[j]) >> 8);
        row[j] = Color::from_argb(row[j]).blend(color).value();
    }
}

}
