/*
 * Copyright (C) 2005-2008 by Pieter Palmers
 *
 * This file is part of FFADO
 * FFADO = Free FireWire (pro-)audio drivers for Linux
 *
 * FFADO is based upon FreeBoB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 * Copied from the jackd/jackdmp sources
 * function names changed in order to avoid naming problems when using this in
 * a jackd backend.
 */

/* Original license:
 * Copyright (C) 2001 Paul Davis
 * Copyright (C) 2004-2006 Grame
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include "PosixThread.h"
#include <string.h> // for memset
#include <errno.h>
#include <assert.h>
#include <sys/prctl.h>

namespace Util
{

IMPL_DEBUG_MODULE( Thread, Thread, DEBUG_LEVEL_NORMAL );

void* PosixThread::ThreadHandler(void* arg)
{
    PosixThread* obj = (PosixThread*)arg;
    RunnableInterface* runnable = obj->fRunnable;
    int err;

    obj->m_lock.Lock();

    // Signal that ThreadHandler has acquired its initial lock
    pthread_mutex_lock(&obj->handler_active_lock);
    obj->handler_active = 1;
    pthread_cond_signal(&obj->handler_active_cond);
    pthread_mutex_unlock(&obj->handler_active_lock);

    if ((err = pthread_setcanceltype(obj->fCancellation, NULL)) != 0) {
        debugError("pthread_setcanceltype err = %s\n", strerror(err));
    }

    // Call Init method
    if (!runnable->Init()) {
        debugError("Thread init fails: thread quits\n");
        obj->m_lock.Unlock();
        return 0;
    }

    std::string threadname = std::string("FW_") + obj->m_id;
    prctl(PR_SET_NAME, threadname.c_str());

    debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) ThreadHandler: start %p\n", obj->m_id.c_str(), obj);

    // If Init succeed start the thread loop
    bool res = true;

    obj->m_lock.Unlock();
    while (obj->fRunning && res) {
        debugOutputExtreme( DEBUG_LEVEL_VERY_VERBOSE, "(%s) ThreadHandler: run %p\n", obj->m_id.c_str(), obj);
        res = runnable->Execute();
        pthread_testcancel();
    }

    debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) ThreadHandler: exit %p\n", obj->m_id.c_str(), obj);
    return 0;
}

int PosixThread::Start()
{
    int res;
    fRunning = true;

    if (fRealTime) {

        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Create RT thread %p with priority %d\n", m_id.c_str(), this, fPriority);

        /* Get the client thread to run as an RT-FIFO
           scheduled thread of appropriate priority.
        */
        pthread_attr_t attributes;
        struct sched_param rt_param;
        pthread_attr_init(&attributes);

        if ((res = pthread_attr_setinheritsched(&attributes, PTHREAD_EXPLICIT_SCHED))) {
            debugError("Cannot request explicit scheduling for RT thread  %d %s\n", res, strerror(res));
            return -1;
        }
        if ((res = pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_JOINABLE))) {
            debugError("Cannot request joinable thread creation for RT thread  %d %s\n", res, strerror(res));
            return -1;
        }
        if ((res = pthread_attr_setscope(&attributes, PTHREAD_SCOPE_SYSTEM))) {
            debugError("Cannot set scheduling scope for RT thread %d %s\n", res, strerror(res));
            return -1;
        }

        if ((res = pthread_attr_setschedpolicy(&attributes, SCHED_FIFO))) {

        //if ((res = pthread_attr_setschedpolicy(&attributes, SCHED_RR))) {
            debugError("Cannot set FIFO scheduling class for RT thread  %d %s\n", res, strerror(res));
            return -1;
        }

        memset(&rt_param, 0, sizeof(rt_param));
        if(fPriority <= 0) {
            debugWarning("Clipping to minimum priority (%d -> 1)\n", fPriority);
            rt_param.sched_priority = 1;
        } else if(fPriority >= 99) {
            debugWarning("Clipping to maximum priority (%d -> 98)\n", fPriority);
            rt_param.sched_priority = 98;
        } else {
            rt_param.sched_priority = fPriority;
        }

        if ((res = pthread_attr_setschedparam(&attributes, &rt_param))) {
            debugError("Cannot set scheduling priority for RT thread %d %s\n", res, strerror(res));
            return -1;
        }

        m_lock.Lock();
        res = pthread_create(&fThread, &attributes, ThreadHandler, this);
        m_lock.Unlock();
        if (res) {
            debugError("Cannot create realtime thread (%d: %s)\n", res, strerror(res));
            debugError(" priority: %d\n", fPriority);
            return -1;
        }
    } else {
        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Create non RT thread %p\n", m_id.c_str(), this);

        m_lock.Lock();
        res = pthread_create(&fThread, 0, ThreadHandler, this);
        m_lock.Unlock();
        if (res) {
            debugError("Cannot create thread %d %s\n", res, strerror(res));
            return -1;
        }
    }

    // Wait for ThreadHandler() to acquire the thread lock (m_lock) before
    // continuing, thereby ensuring that ThreadHandler() acquires a lock on
    // m_lock before anything else tries.
    pthread_mutex_lock(&handler_active_lock);
    while (handler_active == 0)
        pthread_cond_wait(&handler_active_cond, &handler_active_lock);
    pthread_mutex_unlock(&handler_active_lock);

    return 0;
}

int PosixThread::Kill()
{
    if (fThread) { // If thread has been started
        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Kill %p (thread: %p)\n", m_id.c_str(), this, (void *)fThread);
        void* status;
        pthread_cancel(fThread);
        m_lock.Lock();
        pthread_join(fThread, &status);
        m_lock.Unlock();
        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Killed %p (thread: %p)\n", m_id.c_str(), this, (void *)fThread);
        return 0;
    } else {
        return -1;
    }
}

int PosixThread::Stop()
{
    if (fThread) { // If thread has been started
        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Stop %p (thread: %p)\n", m_id.c_str(), this, (void *)fThread);
        void* status;
        fRunning = false; // Request for the thread to stop
        m_lock.Lock();
        pthread_join(fThread, &status);
        fThread = 0;
        m_lock.Unlock();
        debugOutput( DEBUG_LEVEL_VERBOSE, "(%s) Stopped %p (thread: %p)\n", m_id.c_str(), this, (void *)fThread);
        return 0;
    } else {
        return -1;
    }
}

int PosixThread::AcquireRealTime()
{
    struct sched_param rtparam;
    int res;
    debugOutput( DEBUG_LEVEL_VERBOSE, "(%s, %p) Acquire realtime, prio %d\n", m_id.c_str(), this, fPriority);

    if (!fThread)
        return -1;

    memset(&rtparam, 0, sizeof(rtparam));
    if(fPriority <= 0) {
        debugWarning("Clipping to minimum priority (%d -> 1)\n", fPriority);
        rtparam.sched_priority = 1;
    } else if(fPriority >= 99) {
        debugWarning("Clipping to maximum priority (%d -> 98)\n", fPriority);
        rtparam.sched_priority = 98;
    } else {
        rtparam.sched_priority = fPriority;
    }

    //if ((res = pthread_setschedparam(fThread, SCHED_FIFO, &rtparam)) != 0) {

    if ((res = pthread_setschedparam(fThread, SCHED_FIFO, &rtparam)) != 0) {
        debugError("Cannot use real-time scheduling (FIFO/%d) "
                   "(%d: %s)", rtparam.sched_priority, res,
                   strerror(res));
        return -1;
    }
    return 0;
}

int PosixThread::AcquireRealTime(int priority)
{
    fPriority = priority;
    return AcquireRealTime();
}

int PosixThread::DropRealTime()
{
    struct sched_param rtparam;
    int res;
    debugOutput( DEBUG_LEVEL_VERBOSE, "(%s, %p) Drop realtime\n", m_id.c_str(), this);

    if (!fThread)
        return -1;

    memset(&rtparam, 0, sizeof(rtparam));
    rtparam.sched_priority = 0;

    if ((res = pthread_setschedparam(fThread, SCHED_OTHER, &rtparam)) != 0) {
        debugError("Cannot switch to normal scheduling priority(%s)\n", strerror(res));
        return -1;
    }
    return 0;
}

pthread_t PosixThread::GetThreadID()
{
    return fThread;
}

} // end of namespace

