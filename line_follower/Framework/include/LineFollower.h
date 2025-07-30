#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include "Point.h"
#include "minIni.h"

#define LINE_SECTION "Line Follower"

namespace Robot
{
    class LineFollower
    {
    private:
        double m_GoalTurn;
        double m_CurrentTurn;
        double m_P_Gain; // Proportional Gain
        int m_center_pos;

    public:
        LineFollower();
        ~LineFollower();

        void Process(Point2D line_pos, int image_width);
        void LoadINISettings(minIni* ini);

        // Ambil nilai belokan yang sudah dihitung
        double GetTurn() { return m_CurrentTurn; }
    };
}

#endif