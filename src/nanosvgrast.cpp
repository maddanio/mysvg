/*
 * Copyright (c) 2013-14 Mikko Mononen memon@inside.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * The polygon rasterization is heavily based on stb_truetype rasterizer
 * by Sean Barrett - http://nothings.org/
 *
 */

#include <AK/Try.h>
#include <LibGfx/Forward.h>

#include "PathPainter.hpp"
#include "nanosvg.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NSVG__SUBSAMPLES    5
#define NSVG__FIXSHIFT      10
#define NSVG__FIX           (1 << NSVG__FIXSHIFT)
#define NSVG__FIXMASK       (NSVG__FIX-1)
#define NSVG__MEMPAGE_SIZE  1024

struct NSVGedge
{
    float x0,y0,x1,y1;
    int dir;
};

struct NSVGpoint
{
    float x, y;
    float dx, dy;
    float len;
    float dmx, dmy;
    unsigned char flags;
};

struct NSVGactiveEdge {
    int x,dx;
    float ey;
    int dir;
    NSVGactiveEdge(const NSVGedge& e, float startPoint)
    {
        float dxdy = (e.x1 - e.x0) / (e.y1 - e.y0);
        x = (int)floorf(NSVG__FIX * (e.x0 + dxdy * (startPoint - e.y0)));
        dx = int(dxdy < 0 ? -floorf(NSVG__FIX * -dxdy) : floorf(NSVG__FIX * dxdy));
        ey = e.y1;
        dir = e.dir;
    }
};

struct NSVGcachedPaint
{
    char type;
    char spread;
    float xform[6];
    unsigned int colors[256];
};

class NSVGrasterizer
{
    Gfx::PathPainter _painter;

    float tessTol;
    float distTol;

    AK::Vector<NSVGedge> edges;
    AK::Vector<NSVGpoint> points;
    AK::Vector<NSVGpoint> points2;

    int ptEquals(float x1, float y1, float x2, float y2, float tol)
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return dx*dx + dy*dy < tol*tol;
    }

    static float normalize(float *x, float* y)
    {
        float d = sqrtf((*x)*(*x) + (*y)*(*y));
        if (d > 1e-6f) {
            float id = 1.0f / d;
            *x *= id;
            *y *= id;
        }
        return d;
    }

    static float absf(float x) { return x < 0 ? -x : x; }

    void flatten_cubic_bezier(
        float x1, float y1, float x2, float y2,
        float x3, float y3, float x4, float y4,
        int level, bool corner
    )
    {
        float x12,y12,x23,y23,x34,y34,x123,y123,x234,y234,x1234,y1234;
        float dx,dy,d2,d3;

        if (level > 10) return;

        x12 = (x1+x2)*0.5f;
        y12 = (y1+y2)*0.5f;
        x23 = (x2+x3)*0.5f;
        y23 = (y2+y3)*0.5f;
        x34 = (x3+x4)*0.5f;
        y34 = (y3+y4)*0.5f;
        x123 = (x12+x23)*0.5f;
        y123 = (y12+y23)*0.5f;

        dx = x4 - x1;
        dy = y4 - y1;
        d2 = absf(((x2 - x4) * dy - (y2 - y4) * dx));
        d3 = absf(((x3 - x4) * dy - (y3 - y4) * dx));

        if ((d2 + d3)*(d2 + d3) < tessTol * (dx*dx + dy*dy)) {
            _painter.edge_to({x4, y4}, corner);
            return;
        }

        x234 = (x23+x34)*0.5f;
        y234 = (y23+y34)*0.5f;
        x1234 = (x123+x234)*0.5f;
        y1234 = (y123+y234)*0.5f;

        flatten_cubic_bezier(x1,y1, x12,y12, x123,y123, x1234,y1234, level+1, false);
        flatten_cubic_bezier(x1234,y1234, x234,y234, x34,y34, x4,y4, level+1, corner);
    }

    void flatten_shape(const NSVGshape& shape, float scale)
    {
        auto convert_paint = [](const NSVGpaint& paint)
        {
            int color = 0xffffff;
            switch(paint.type)
            {
            case NSVG_PAINT_COLOR:
                color = paint.color;
                break;
            case NSVG_PAINT_LINEAR_GRADIENT:
                color = paint.gradient->stops[0].color;
                break;
            }
            return Gfx::Rasterizer::Paint{
                .color = Gfx::Rasterizer::color_t{
                    static_cast<unsigned char>(color & 0xff),
                    static_cast<unsigned char>((color >> 8) & 0xff),
                    static_cast<unsigned char>((color >> 16) & 0xff),
                    static_cast<unsigned char>((color >> 24) & 0xff),
                }
            };
        };
        for (auto path = shape.paths; path != NULL; path = path->next)
        {
            _painter.begin(
                {path->pts[0]*scale, path->pts[1]*scale},
                (
                    shape.stroke.type != NSVG_PAINT_NONE ?
                    (
                        path->closed ?
                        Gfx::PathPainter::StrokeKind::ClosedStroke :
                        Gfx::PathPainter::StrokeKind::OpenStroke
                    ) :
                    Gfx::PathPainter::StrokeKind::NoStroke
                ),
                shape.fill.type != NSVG_PAINT_NONE ? Gfx::PathPainter::FillKind::Filled : Gfx::PathPainter::FillKind::NotFilled,
                shape.strokeWidth
            );
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];
                flatten_cubic_bezier(p[0]*scale,p[1]*scale, p[2]*scale,p[3]*scale, p[4]*scale,p[5]*scale, p[6]*scale,p[7]*scale, 0, false);
            }
            _painter.end(convert_paint(shape.stroke), convert_paint(shape.fill), false);
        }
    }

public:

    NSVGrasterizer(Gfx::Rasterizer::image_t image)
    : _painter{image}
    {
        tessTol = 0.25f;
        distTol = 0.01f;
    }

    ~NSVGrasterizer()
    {
    }

    //   r - pointer to rasterizer context
    //   image - pointer to image to rasterize
    //   tx,ty - image offset (applied after scaling)
    //   scale - image scale
    //   dst - pointer to destination image data, 4 bytes per pixel (RGBA)
    //   w - width of the image to render
    //   h - height of the image to render
    //   stride - number of bytes per scaleline in the destination buffer
    void run(
        NSVGimage* image,
        float scale
    )
    {
        for (auto shape = image->shapes; shape != NULL; shape = shape->next)
        {
            if (!(shape->flags & NSVG_FLAGS_VISIBLE))
                continue;
            flatten_shape(
                *shape,
                scale
            );
        }
    }
};

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <LibGfx/Bitmap.h>
#include <LibGfx/PNGWriter.h>
#include "stdio.h"

int main(int argc, const char** argv)
{
    auto svg = nsvgParseFromFile(argv[1], "px", 96);
    size_t scale = 4;
    size_t w = svg->width * scale;
    size_t h = svg->height * scale;
    auto image = MUST(Gfx::Bitmap::try_create(Gfx::BitmapFormat::BGRA8888, Gfx::IntSize{w, h}));
    NSVGrasterizer rasterizer{image};
    rasterizer.run(svg, scale);
    auto encoded = Gfx::PNGWriter::encode(image);
    auto out = fopen("svg.png", "w");
    fwrite(encoded.data(), encoded.size(), 1, out);
    fclose(out);
    return 0;
}
