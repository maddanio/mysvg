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

#include "Rasterizer.hpp"
#include "StrokePainter.hpp"
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
    Gfx::Rasterizer _rasterizer;

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

    void addPathPoint(float x, float y, int flags)
    {
        if (!points.is_empty()) {
            NSVGpoint* pt = &points.last();
            if (ptEquals(pt->x,pt->y, x,y, distTol)) {
                pt->flags = (unsigned char)(pt->flags | flags);
                return;
            }
        }

        points.append({.x = x, .y = y, .flags = (unsigned char)flags});
    }

    void appendPathPoint(NSVGpoint pt)
    {
        points.append(pt);
    }

    void duplicatePoints()
    {
        points2 = points;
    }

    void addEdge(float x0, float y0, float x1, float y1)
    {
        _rasterizer.add_edge({{x0, y0}, {x1, y1}});
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
        int level, int type
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
            addPathPoint(x4, y4, type);
            return;
        }

        x234 = (x23+x34)*0.5f;
        y234 = (y23+y34)*0.5f;
        x1234 = (x123+x234)*0.5f;
        y1234 = (y123+y234)*0.5f;

        flatten_cubic_bezier(x1,y1, x12,y12, x123,y123, x1234,y1234, level+1, 0);
        flatten_cubic_bezier(x1234,y1234, x234,y234, x34,y34, x4,y4, level+1, type);
    }

    void flatten_shape(const NSVGshape& shape, float scale)
    {
        for (auto path = shape.paths; path != NULL; path = path->next)
        {
            points.clear();
            // Flatten path
            addPathPoint(path->pts[0]*scale, path->pts[1]*scale, 0);
            for (int i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];
                flatten_cubic_bezier(p[0]*scale,p[1]*scale, p[2]*scale,p[3]*scale, p[4]*scale,p[5]*scale, p[6]*scale,p[7]*scale, 0, 0);
            }
            // Close path
            addPathPoint(path->pts[0]*scale, path->pts[1]*scale, 0);
            // Build edges
            for (int i = 0, j = points.size()-1; i < points.size(); j = i++)
                addEdge(points[j].x, points[j].y, points[i].x, points[i].y);
        }
    }

    enum NSVGpointFlags
    {
        NSVG_PT_CORNER = 0x01,
        NSVG_PT_BEVEL = 0x02,
        NSVG_PT_LEFT = 0x04
    };

    static void init_closed(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p0, NSVGpoint* p1, float lineWidth)
    {
        float w = lineWidth * 0.5f;
        float dx = p1->x - p0->x;
        float dy = p1->y - p0->y;
        float len = normalize(&dx, &dy);
        float px = p0->x + dx*len*0.5f, py = p0->y + dy*len*0.5f;
        float dlx = dy, dly = -dx;
        float lx = px - dlx*w, ly = py - dly*w;
        float rx = px + dlx*w, ry = py + dly*w;
        left.x = lx;
        left.y = ly;
        right.x = rx;
        right.y = ry;
    }

    void buttCap(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p, float dx, float dy, float lineWidth, int connect)
    {
        float w = lineWidth * 0.5f;
        float px = p->x, py = p->y;
        float dlx = dy, dly = -dx;
        float lx = px - dlx*w, ly = py - dly*w;
        float rx = px + dlx*w, ry = py + dly*w;

        addEdge(lx, ly, rx, ry);

        if (connect) {
            addEdge(left.x, left.y, lx, ly);
            addEdge(rx, ry, right.x, right.y);
        }
        left.x = lx; left.y = ly;
        right.x = rx; right.y = ry;
    }

    void squareCap(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p, float dx, float dy, float lineWidth, int connect)
    {
        float w = lineWidth * 0.5f;
        float px = p->x - dx*w, py = p->y - dy*w;
        float dlx = dy, dly = -dx;
        float lx = px - dlx*w, ly = py - dly*w;
        float rx = px + dlx*w, ry = py + dly*w;

        addEdge(lx, ly, rx, ry);

        if (connect) {
            addEdge(left.x, left.y, lx, ly);
            addEdge(rx, ry, right.x, right.y);
        }
        left.x = lx; left.y = ly;
        right.x = rx; right.y = ry;
    }

    #ifndef NSVG_PI
    #define NSVG_PI (3.14159265358979323846264338327f)
    #endif

    void roundCap(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p, float dx, float dy, float lineWidth, int ncap, int connect)
    {
        int i;
        float w = lineWidth * 0.5f;
        float px = p->x, py = p->y;
        float dlx = dy, dly = -dx;
        float lx = 0, ly = 0, rx = 0, ry = 0, prevx = 0, prevy = 0;

        for (i = 0; i < ncap; i++) {
            float a = (float)i/(float)(ncap-1)*NSVG_PI;
            float ax = cosf(a) * w, ay = sinf(a) * w;
            float x = px - dlx*ax - dx*ay;
            float y = py - dly*ax - dy*ay;

            if (i > 0)
                addEdge(prevx, prevy, x, y);

            prevx = x;
            prevy = y;

            if (i == 0) {
                lx = x; ly = y;
            } else if (i == ncap-1) {
                rx = x; ry = y;
            }
        }

        if (connect) {
            addEdge(left.x, left.y, lx, ly);
            addEdge(rx, ry, right.x, right.y);
        }

        left.x = lx; left.y = ly;
        right.x = rx; right.y = ry;
    }

    void bevelJoin(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p0, NSVGpoint* p1, float lineWidth)
    {
        float w = lineWidth * 0.5f;
        float dlx0 = p0->dy, dly0 = -p0->dx;
        float dlx1 = p1->dy, dly1 = -p1->dx;
        float lx0 = p1->x - (dlx0 * w), ly0 = p1->y - (dly0 * w);
        float rx0 = p1->x + (dlx0 * w), ry0 = p1->y + (dly0 * w);
        float lx1 = p1->x - (dlx1 * w), ly1 = p1->y - (dly1 * w);
        float rx1 = p1->x + (dlx1 * w), ry1 = p1->y + (dly1 * w);

        addEdge(lx0, ly0, left.x, left.y);
        addEdge(lx1, ly1, lx0, ly0);

        addEdge(right.x, right.y, rx0, ry0);
        addEdge(rx0, ry0, rx1, ry1);

        left.x = lx1; left.y = ly1;
        right.x = rx1; right.y = ry1;
    }

    void miterJoin(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p0, NSVGpoint* p1, float lineWidth)
    {
        float w = lineWidth * 0.5f;
        float dlx0 = p0->dy, dly0 = -p0->dx;
        float dlx1 = p1->dy, dly1 = -p1->dx;
        float lx0, rx0, lx1, rx1;
        float ly0, ry0, ly1, ry1;

        if (p1->flags & NSVG_PT_LEFT) {
            lx0 = lx1 = p1->x - p1->dmx * w;
            ly0 = ly1 = p1->y - p1->dmy * w;
            addEdge(lx1, ly1, left.x, left.y);

            rx0 = p1->x + (dlx0 * w);
            ry0 = p1->y + (dly0 * w);
            rx1 = p1->x + (dlx1 * w);
            ry1 = p1->y + (dly1 * w);
            addEdge(right.x, right.y, rx0, ry0);
            addEdge(rx0, ry0, rx1, ry1);
        } else {
            lx0 = p1->x - (dlx0 * w);
            ly0 = p1->y - (dly0 * w);
            lx1 = p1->x - (dlx1 * w);
            ly1 = p1->y - (dly1 * w);
            addEdge(lx0, ly0, left.x, left.y);
            addEdge(lx1, ly1, lx0, ly0);

            rx0 = rx1 = p1->x + p1->dmx * w;
            ry0 = ry1 = p1->y + p1->dmy * w;
            addEdge(right.x, right.y, rx1, ry1);
        }

        left.x = lx1; left.y = ly1;
        right.x = rx1; right.y = ry1;
    }

    void roundJoin(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p0, NSVGpoint* p1, float lineWidth, int ncap)
    {
        int i, n;
        float w = lineWidth * 0.5f;
        float dlx0 = p0->dy, dly0 = -p0->dx;
        float dlx1 = p1->dy, dly1 = -p1->dx;
        float a0 = atan2f(dly0, dlx0);
        float a1 = atan2f(dly1, dlx1);
        float da = a1 - a0;
        float lx, ly, rx, ry;

        if (da < NSVG_PI) da += NSVG_PI*2;
        if (da > NSVG_PI) da -= NSVG_PI*2;

        n = (int)ceilf((absf(da) / NSVG_PI) * (float)ncap);
        if (n < 2) n = 2;
        if (n > ncap) n = ncap;

        lx = left.x;
        ly = left.y;
        rx = right.x;
        ry = right.y;

        for (i = 0; i < n; i++) {
            float u = (float)i/(float)(n-1);
            float a = a0 + u*da;
            float ax = cosf(a) * w, ay = sinf(a) * w;
            float lx1 = p1->x - ax, ly1 = p1->y - ay;
            float rx1 = p1->x + ax, ry1 = p1->y + ay;

            addEdge(lx1, ly1, lx, ly);
            addEdge(rx, ry, rx1, ry1);

            lx = lx1; ly = ly1;
            rx = rx1; ry = ry1;
        }

        left.x = lx; left.y = ly;
        right.x = rx; right.y = ry;
    }

    void straightJoin(NSVGpoint& left, NSVGpoint& right, NSVGpoint* p1, float lineWidth)
    {
        float w = lineWidth * 0.5f;
        float lx = p1->x - (p1->dmx * w), ly = p1->y - (p1->dmy * w);
        float rx = p1->x + (p1->dmx * w), ry = p1->y + (p1->dmy * w);

        addEdge(lx, ly, left.x, left.y);
        addEdge(right.x, right.y, rx, ry);

        left.x = lx; left.y = ly;
        right.x = rx; right.y = ry;
    }

    int curveDivs(float r, float arc, float tol)
    {
        float da = acosf(r / (r + tol)) * 2.0f;
        int divs = (int)ceilf(arc / da);
        if (divs < 2) divs = 2;
        return divs;
    }

    void expandStroke(int closed, int lineJoin, int lineCap, float lineWidth)
    {
        int ncap = curveDivs(lineWidth*0.5f, NSVG_PI, tessTol); // Calculate divisions per half circle.
        NSVGpoint left = {0,0,0,0,0,0,0,0}, right = {0,0,0,0,0,0,0,0}, firstLeft = {0,0,0,0,0,0,0,0}, firstRight = {0,0,0,0,0,0,0,0};
        NSVGpoint* p0, *p1;
        int j, s, e;

        // Build stroke edges
        if (closed) {
            // Looping
            p0 = &points.last();
            p1 = &points.first();
            s = 0;
            e = points.size();
        } else {
            // Add cap
            p0 = &points.first();
            p1 = &points[1];
            s = 1;
            e = points.size()-1;
        }

        if (closed) {
            init_closed(left, right, p0, p1, lineWidth);
            firstLeft = left;
            firstRight = right;
        } else {
            // Add cap
            float dx = p1->x - p0->x;
            float dy = p1->y - p0->y;
            normalize(&dx, &dy);
            if (lineCap == NSVG_CAP_BUTT)
                buttCap(left, right, p0, dx, dy, lineWidth, 0);
            else if (lineCap == NSVG_CAP_SQUARE)
                squareCap(left, right, p0, dx, dy, lineWidth, 0);
            else if (lineCap == NSVG_CAP_ROUND)
                roundCap(left, right, p0, dx, dy, lineWidth, ncap, 0);
        }

        for (j = s; j < e; ++j) {
            if (p1->flags & NSVG_PT_CORNER) {
                if (lineJoin == NSVG_JOIN_ROUND)
                    roundJoin(left, right, p0, p1, lineWidth, ncap);
                else if (lineJoin == NSVG_JOIN_BEVEL || (p1->flags & NSVG_PT_BEVEL))
                    bevelJoin(left, right, p0, p1, lineWidth);
                else
                    miterJoin(left, right, p0, p1, lineWidth);
            } else {
                straightJoin(left, right, p1, lineWidth);
            }
            p0 = p1++;
        }

        if (closed) {
            // Loop it
            addEdge(firstLeft.x, firstLeft.y, left.x, left.y);
            addEdge(right.x, right.y, firstRight.x, firstRight.y);
        } else {
            // Add cap
            float dx = p1->x - p0->x;
            float dy = p1->y - p0->y;
            normalize(&dx, &dy);
            if (lineCap == NSVG_CAP_BUTT)
                buttCap(right, left, p1, -dx, -dy, lineWidth, 1);
            else if (lineCap == NSVG_CAP_SQUARE)
                squareCap(right, left, p1, -dx, -dy, lineWidth, 1);
            else if (lineCap == NSVG_CAP_ROUND)
                roundCap(right, left, p1, -dx, -dy, lineWidth, ncap, 1);
        }
    }

    void prepareStroke(float miterLimit, int lineJoin)
    {
        int i, j;
        NSVGpoint* p0, *p1;

        p0 = &points.last();
        p1 = &points.first();
        for (i = 0; i < points.size(); i++) {
            // Calculate segment direction and length
            p0->dx = p1->x - p0->x;
            p0->dy = p1->y - p0->y;
            p0->len = normalize(&p0->dx, &p0->dy);
            // Advance
            p0 = p1++;
        }

        // calculate joins
        p0 = &points.last();
        p1 = &points.first();
        for (j = 0; j < points.size(); j++) {
            float dlx0, dly0, dlx1, dly1, dmr2, cross;
            dlx0 = p0->dy;
            dly0 = -p0->dx;
            dlx1 = p1->dy;
            dly1 = -p1->dx;
            // Calculate extrusions
            p1->dmx = (dlx0 + dlx1) * 0.5f;
            p1->dmy = (dly0 + dly1) * 0.5f;
            dmr2 = p1->dmx*p1->dmx + p1->dmy*p1->dmy;
            if (dmr2 > 0.000001f) {
                float s2 = 1.0f / dmr2;
                if (s2 > 600.0f) {
                    s2 = 600.0f;
                }
                p1->dmx *= s2;
                p1->dmy *= s2;
            }

            // Clear flags, but keep the corner.
            p1->flags = (p1->flags & NSVG_PT_CORNER) ? NSVG_PT_CORNER : 0;

            // Keep track of left turns.
            cross = p1->dx * p0->dy - p0->dx * p1->dy;
            if (cross > 0.0f)
                p1->flags |= NSVG_PT_LEFT;

            // Check to see if the corner needs to be beveled.
            if (p1->flags & NSVG_PT_CORNER) {
                if ((dmr2 * miterLimit*miterLimit) < 1.0f || lineJoin == NSVG_JOIN_BEVEL || lineJoin == NSVG_JOIN_ROUND) {
                    p1->flags |= NSVG_PT_BEVEL;
                }
            }

            p0 = p1++;
        }
    }

    void flatten_shapeStroke(const NSVGshape& shape, float scale)
    {
        int i, j, closed;
        NSVGpath* path;
        NSVGpoint* p0, *p1;
        float miterLimit = shape.miterLimit;
        int lineJoin = shape.strokeLineJoin;
        int lineCap = shape.strokeLineCap;
        float lineWidth = shape.strokeWidth * scale;

        for (path = shape.paths; path != NULL; path = path->next) {
            // Flatten path
            points.clear();
            addPathPoint(path->pts[0]*scale, path->pts[1]*scale, NSVG_PT_CORNER);
            for (i = 0; i < path->npts-1; i += 3) {
                float* p = &path->pts[i*2];
                flatten_cubic_bezier(p[0]*scale,p[1]*scale, p[2]*scale,p[3]*scale, p[4]*scale,p[5]*scale, p[6]*scale,p[7]*scale, 0, NSVG_PT_CORNER);
            }
            if (points.size() < 2)
                continue;

            closed = path->closed;

            // If the first and last points are the same, remove the last, mark as closed path.
            p0 = &points.last();
            p1 = &points.first();
            if (ptEquals(p0->x,p0->y, p1->x,p1->y, distTol)) {
                points.take_last();
                p0 = &points.last();
                closed = 1;
            }

            if (shape.strokeDashCount > 0) {
                int idash = 0, dashState = 1;
                float totalDist = 0, dashLen, allDashLen, dashOffset;
                NSVGpoint cur;

                if (closed)
                    appendPathPoint(points.first());

                // Duplicate points -> points2.
                duplicatePoints();

                points.clear();
                cur = points2[0];
                appendPathPoint(cur);

                // Figure out dash offset.
                allDashLen = 0;
                for (j = 0; j < shape.strokeDashCount; j++)
                    allDashLen += shape.strokeDashArray[j];
                if (shape.strokeDashCount & 1)
                    allDashLen *= 2.0f;
                // Find location inside pattern
                dashOffset = fmodf(shape.strokeDashOffset, allDashLen);
                if (dashOffset < 0.0f)
                    dashOffset += allDashLen;

                while (dashOffset > shape.strokeDashArray[idash]) {
                    dashOffset -= shape.strokeDashArray[idash];
                    idash = (idash + 1) % shape.strokeDashCount;
                }
                dashLen = (shape.strokeDashArray[idash] - dashOffset) * scale;

                for (j = 1; j < points2.size(); ) {
                    float dx = points2[j].x - cur.x;
                    float dy = points2[j].y - cur.y;
                    float dist = sqrtf(dx*dx + dy*dy);

                    if ((totalDist + dist) > dashLen) {
                        // Calculate intermediate point
                        float d = (dashLen - totalDist) / dist;
                        float x = cur.x + dx * d;
                        float y = cur.y + dy * d;
                        addPathPoint(x, y, NSVG_PT_CORNER);

                        // Stroke
                        if (points.size() > 1 && dashState) {
                            prepareStroke(miterLimit, lineJoin);
                            expandStroke(0, lineJoin, lineCap, lineWidth);
                        }
                        // Advance dash pattern
                        dashState = !dashState;
                        idash = (idash+1) % shape.strokeDashCount;
                        dashLen = shape.strokeDashArray[idash] * scale;
                        // Restart
                        cur.x = x;
                        cur.y = y;
                        cur.flags = NSVG_PT_CORNER;
                        totalDist = 0.0f;
                        points.clear();
                        appendPathPoint(cur);
                    } else {
                        totalDist += dist;
                        cur = points2[j];
                        appendPathPoint(cur);
                        j++;
                    }
                }
                // Stroke any leftover path
                if (points.size() > 1 && dashState)
                    expandStroke(0, lineJoin, lineCap, lineWidth);
            } else {
                prepareStroke(miterLimit, lineJoin);
                expandStroke(closed, lineJoin, lineCap, lineWidth);
            }
        }
    }


public:

    NSVGrasterizer(Gfx::Rasterizer::image_t image)
    : _rasterizer{image}
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
            if (shape->fill.type != NSVG_PAINT_NONE)
            {
                flatten_shape(*shape, scale);
                _rasterizer.rasterize_edges(
                    shape->fillRule == NSVG_FILLRULE_NONZERO ? Gfx::Rasterizer::FillRule::nonzero : Gfx::Rasterizer::FillRule::evenodd,
                    convert_paint(shape->fill)
                );
            }
            if (shape->stroke.type != NSVG_PAINT_NONE && (shape->strokeWidth * scale) > 0.01f)
            {
                flatten_shapeStroke(*shape, scale);
                _rasterizer.rasterize_edges(
                    Gfx::Rasterizer::FillRule::nonzero,
                    convert_paint(shape->stroke)
                );
            }
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
