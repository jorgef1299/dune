//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Pedro Calado                                                     *
//***************************************************************************

#ifndef SUPERVISOR_VEHICLE_MANEUVERSUPERVISOR_HPP_INCLUDED_
#define SUPERVISOR_VEHICLE_MANEUVERSUPERVISOR_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <queue>

// DUNE headers.
#include <DUNE/IMC.hpp>
#include <DUNE/Time.hpp>
#include <DUNE/Memory.hpp>
#include <DUNE/Tasks.hpp>

// Local headers
#include "Request.hpp"

namespace Supervisors
{
  namespace Vehicle
  {
    using DUNE_NAMESPACES;

    class ManeuverSupervisor
    {
    public:
      ManeuverSupervisor(Tasks::Task* task, double timeout):
        m_task(task),
        m_curr_req(NULL),
        m_timeout(timeout),
        m_valid_state(false)
      { }

      ~ManeuverSupervisor(void)
      {
        clearCurrent();

        while (!m_reqs.empty())
        {
          delete m_reqs.front();
          m_reqs.pop();
        }
      }

      //! Add a request for stopping a maneuver
      void
      addStop(void)
      {
        Request* req = new Request(RT_STOP);
        m_reqs.push(req);

        m_task->inf("added stop");

        processRequests();
      }

      //! Add a request for starting a maneuver
      void
      addStart(const IMC::Message* msg)
      {
        Request* req = new Request(RT_START, msg);
        m_reqs.push(req);

        m_task->inf("added start");

        processRequests();
      }

      //! Update current requests with ManeuverControlState message
      void
      update(const IMC::ManeuverControlState* msg)
      {
        m_state = msg->state;
        m_valid_state = true;

        if (!isProcessing())
        {
          processRequests();
          return;
        }

        switch (msg->state)
        {
          case IMC::ManeuverControlState::MCS_EXECUTING:
            if (m_curr_req->isStart())
              m_task->inf("maneuver started");
            else
              m_task->err("request doesn't match");
            break;
          case IMC::ManeuverControlState::MCS_STOPPED:
            if (m_curr_req->isStop())
              m_task->inf("maneuver stopped");
            else
              m_task->err("request doesn't match");
            break;
          case IMC::ManeuverControlState::MCS_DONE:
          case IMC::ManeuverControlState::MCS_ERROR:
            m_task->war("request dropped");
            break;
        }

        clearCurrent();
        processRequests();
      }

      //! Check if current request hasn't expired
      void
      checkExpiration(void)
      {
        if (!isExpired())
          return;

        m_task->err("reply to maneuver %s has timed out", m_curr_req->isStop() ? "stop" : "start");
        m_task->err("system may need maintenance");

        // restart timer
        m_curr_req->issue();
      }

    private:
      //! Process requests in the queue
      void
      processRequests(void)
      {
        if (isProcessing())
          return;

        if (!m_reqs.size())
          return;

        m_curr_req = m_reqs.front();
        m_reqs.pop();

        if (m_valid_state)
        {
          // Two stops of two starts in a row generate error
          if (m_curr_req->isStop())
          {
            if (m_state != IMC::ManeuverControlState::MCS_EXECUTING)
            {
              m_task->inf("got StopManeuver, but no maneuver is executing");
              clearCurrent();
              processRequests();
              return;
            }

            if (m_reqs.size())
            {
              // Two stops in a row?
              if (m_reqs.front()->isStop())
              {
                m_task->inf("got two StopManeuver in a row");
                // clear this and use next one
                clearCurrent();
                processRequests();
                return;
              }
            }
          }
          else if (m_curr_req->isStart())
          {
            if (m_state == IMC::ManeuverControlState::MCS_EXECUTING)
            {
              m_task->err("already executing, cannot start without stopping");
              // clear this one and use next one
              clearCurrent();
              processRequests();
              return;
            }

            if (m_reqs.size())
            {
              // Two starts in a row
              if (m_reqs.front()->isStart())
              {
                m_task->err("got two or more maneuvers in a row, ignoring oldest");
                // clear this and use next one
                clearCurrent();
                processRequests();
                return;
              }
              else if (m_reqs.front()->isStop())
              {
                // if a stop is coming, ignore this maneuver
                m_task->war("a stop comes right after, ignoring this maneuver");
                // clear this one and pop next one
                clearCurrent();
                m_reqs.pop();
                processRequests();
                return;
              }
              else
              {
                m_task->err("undefined state");
              }
            }
          }
          else
          {
            m_task->err("undefined state");
          }
        }
        else if (m_curr_req->isStop())
        {
          clearCurrent();
          return;
        }

        m_curr_req->issue();
        m_task->dispatch(m_curr_req->getMessage());
        m_task->inf("dispatched %s request", m_curr_req->isStop() ? "stop" : "start");
      }

      //! Check if a request is being processed
      bool
      isProcessing(void)
      {
        return (m_curr_req != NULL);
      }

      //! Check if current request has expired
      bool
      isExpired(void)
      {
        if (isProcessing())
        {
          if (Time::Clock::get() - m_curr_req->getIssueTime() > m_timeout)
            return true;
          else
            return false;
        }
        else
        {
          return false;
        }
      }

      //! Clear current request
      void
      clearCurrent(void)
      {
        if (isProcessing())
          Memory::clear(m_curr_req);
      }

      //! Pointer to task
      Tasks::Task* m_task;
      //! Pointer to current request being handled
      Request* m_curr_req;
      //! Timeout for a request
      double m_timeout;
      //! Queue of requests
      std::queue<Request*> m_reqs;
      //! True if maneuver control state has been set
      bool m_valid_state;
      //! Current maneuver control state
      unsigned m_state;
    };
  }
}

#endif
