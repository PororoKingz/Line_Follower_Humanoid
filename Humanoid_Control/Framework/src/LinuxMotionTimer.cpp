/*
 * LinuxMotionTimer.cpp
 *
 * Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "LinuxMotionTimer.h"

using namespace Robot;

LinuxMotionTimer::LinuxMotionTimer(MotionManager* manager)
{
    m_Manager = manager;
    m_TimerRunning = false;
    m_FinishTimer = false;
    m_Interval_ns = 8000000; // 8ms
}

LinuxMotionTimer::~LinuxMotionTimer()
{
    Stop();
}

void *LinuxMotionTimer::TimerProc(void *param)
{
    LinuxMotionTimer *timer = (LinuxMotionTimer *)param;
    struct timespec next_time;
    struct timespec current_time;

    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while(!timer->m_FinishTimer)
    {
        next_time.tv_nsec += timer->m_Interval_ns;
        if(next_time.tv_nsec >= 1000000000)
        {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
        
        if(timer->m_Manager != 0)
            timer->m_Manager->Process();

        clock_gettime(CLOCK_MONOTONIC, &current_time);
    }
    
    pthread_exit(NULL);
    return NULL;
}

void LinuxMotionTimer::Start()
{
    if(m_TimerRunning == true)
        return;

    m_FinishTimer = false;
    if(pthread_create(&m_Thread, NULL, TimerProc, this) != 0)
    {
        fprintf(stderr, "Failed to create motion timer thread!\n");
        return;
    }
    m_TimerRunning = true;
}

void LinuxMotionTimer::Stop()
{
    if(m_TimerRunning == false)
        return;

    m_FinishTimer = true;
    pthread_join(m_Thread, NULL);
    m_TimerRunning = false;
}

bool LinuxMotionTimer::IsRunning()
{
    return m_TimerRunning;
}