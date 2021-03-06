/* -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/*
 *  Main authors:
 *     Guido Tack <tack@gecode.org>
 *
 *  Copyright:
 *     Guido Tack, 2012
 *
 *  Last modified:
 *     $Date: 2017-03-28 16:18:06 +0200 (Di, 28 Mär 2017) $ by $Author: schulte $
 *     $Revision: 15620 $
 *
 *  This file is part of Gecode, the generic constraint
 *  development environment:
 *     http://www.gecode.org
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 *  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include "rbs.hh"
#include "../../TransportNetwork.hpp"

namespace Gecode { namespace Search { namespace Meta {

  bool
  RestartStop::stop(const Statistics& s, const Options& o) {
    // Stop if the fail limit for the engine says so
    if (s.fail > l) {
      e_stopped = true;
      m_stat.restart++;
      return true;
    }
    // Stop if the stop object for the meta engine says so
    if ((m_stop != NULL) && m_stop->stop(m_stat+s,o)) {
      e_stopped = false;
      return true;
    }
    return false;
  }


  Space*
  TemplRBS::next(void) {
    using namespace templ::solvers::csp;
    if (restart) {
      restart = false;
      sslr++;
      NoGoods& ng = e->nogoods();
      // Reset number of no-goods found
      ng.ng(0);
      // restart: number of restarts
      // sslr: number of solutions since last restart
      // fail: number of fails since last restart
      // last: last space
      // ng: nogoods
      MetaInfo mi(stop->m_stat.restart,sslr,e->statistics().fail,last,ng);
      bool r = master->master(mi);
      stop->m_stat.nogood += ng.ng();
      if (master->status(stop->m_stat) == SS_FAILED) {
        stop->update(e->statistics());
        delete master;
        master = NULL;
        e->reset(NULL);
        return NULL;
      } else if (r) {
        stop->update(e->statistics());
        Space* slave = master;
        master = master->clone(shared_data,shared_info);
        complete = slave->slave(mi);
        TransportNetwork* slaveTn = dynamic_cast<TransportNetwork*>(slave);
        if(slaveTn)
        {
            TransportNetwork* tn = dynamic_cast<TransportNetwork*>(master);
            slaveTn->setCurrentMaster(tn);
        }
        e->reset(slave);
        sslr = 0;
        stop->m_stat.restart++;
      }
    }
    while (true) {
      Space* n = e->next();
      if (n != NULL) {
        // The engine found a solution
        restart = true;
        delete last;
        last = n->clone(shared_data);
        return n;
      } else if ( (!complete && !e->stopped()) ||
                  (e->stopped() && stop->enginestopped()) ) {
        // The engine must perform a true restart
        // The number of the restart has been incremented in the stop object
        sslr = 0;
        NoGoods& ng = e->nogoods();
        ng.ng(0);
        MetaInfo mi(stop->m_stat.restart,sslr,e->statistics().fail,last,ng);
        (void) master->master(mi);
        stop->m_stat.nogood += ng.ng();
        long unsigned int nl = ++(*co);
        stop->limit(e->statistics(),nl);
        if (master->status(stop->m_stat) == SS_FAILED)
          return NULL;
        Space* slave = master;
        master = master->clone(shared_data,shared_info);
        TransportNetwork* slaveTn = dynamic_cast<TransportNetwork*>(slave);
        if(slaveTn)
        {
            TransportNetwork* tn = dynamic_cast<TransportNetwork*>(master);
            slaveTn->setCurrentMaster(tn);
        }
        complete = slave->slave(mi);
        e->reset(slave);
      } else {
        return NULL;
      }
    }
    GECODE_NEVER;
    return NULL;
  }

  Search::Statistics
  TemplRBS::statistics(void) const {
    return stop->metastatistics()+e->statistics();
  }

  void
  TemplRBS::constrain(const Space& b) {
    if (!best)
      throw NoBest("TemplRBS::constrain");
    if (last != NULL) {
      last->constrain(b);
      if (last->status() == SS_FAILED) {
        delete last;
      } else {
        return;
      }
    }
    last = b.clone(shared_data);
    master->constrain(b);
    e->constrain(b);
  }

  bool
  TemplRBS::stopped(void) const {
    /*
     * What might happen during parallel search is that the
     * engine has been stopped but the meta engine has not, so
     * the meta engine does not perform a restart. However the
     * invocation of next will do so and no restart will be
     * missed.
     */
    return e->stopped();
  }

  TemplRBS::~TemplRBS(void) {
    delete e;
    delete master;
    delete last;
    delete co;
    delete stop;
  }

}}}

// STATISTICS: search-meta
