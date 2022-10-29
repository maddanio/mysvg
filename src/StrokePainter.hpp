#include "rasterizer.hpp"

class StrokePainter
{
public:
    enum class LineEnd {Start, End};
    enum class CapType {Butt, Square, Round};
    enum class JoinType {Bevel, Miter, Round, Straight};
    StrokePainter(Rasterizer::image_t image)
    : m_rasterizer{image}
    {
    }
    void begin(Rasterizer::point_t p, float thickness)
    {
        m_thickness = thickness;
        m_current_point = p;
    }
    void stroke_to(Rasterizer::point_t p, bool corner)
    {
        m_direction = (p - m_current_point).normalized();
        auto offset = m_direction.orthogonal() * m_thickness;
        auto left = m_current_point + offset;
        auto right = m_current_point + offset;
        add_edge({left, left + m_direction});
        add_edge({right, right + m_direction});
        if (m_first)
            add_cap(LineEnd::Start);
        m_current_point = p;
        m_first = false;
    }
    void end()
    {
        add_cap(LineEnd::End);
    }
private:
    void add_edge(Rasterizer::Edge edge)
    {
        m_rasterizer.add_edge(edge);
    }
    void add_cap(LineEnd end)
    {
        auto direction = m_direction;
        if (end == LineEnd::End)
            direction = direction * -1;
        auto offset = direction.orthogonal() * m_thickness;
        auto left = m_current_point + offset;
        auto right = m_current_point + offset;
        switch(m_cap_type)
        {
        case CapType::Butt:
            add_edge({left, right});
            break;
        case CapType::Square:
        {
            auto extension = direction * 0.5f;
            auto left_end = left + extension;
            auto right_end = right + extension;
            add_edge({left, left_end});
            add_edge({right, right_end});
            add_edge({left_end, right_end});
            break;
        }
        }
    }
    Rasterizer m_rasterizer;
    float m_thickness = 1;
    bool m_first = true;
    CapType m_cap_type;
    JoinType m_join_type;
    Rasterizer::point_t m_direction;
    Rasterizer::point_t m_current_point;
};
