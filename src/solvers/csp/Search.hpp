#ifndef TEMPL_SOLVERS_CSP_SEARCH_HPP
#define TEMPL_SOLVERS_CSP_SEARCH_HPP

#include <gecode/kernel.hh>
#include <gecode/search.hh>

//namespace Gecode {
//
//  /**
//   * \brief Meta-engine performing restart-based search
//   *
//   * The engine uses the Cutoff sequence supplied in the options \a o to
//   * periodically restart the search of engine \a E.
//   *
//   * The class \a T can implement member functions
//   * \code virtual bool master(const MetaInfo& mi) \endcode
//   * and
//   * \code virtual bool slave(const MetaInfo& mi) \endcode
//   *
//   * Whenever exploration restarts or a solution is found, the
//   * engine executes the functions on the master and slave
//   * space. For more details, consult "Modeling and Programming
//   * with Gecode".
//   *
//   * \ingroup TaskModelSearch
//   */
//  template<class T, template<class> class E = DFS>
//  class TemplRBS : public Search::Base<T> {
//    using Search::Base<T>::e;
//  public:
//    /// Initialize engine for space \a s and options \a o
//    TemplRBS(T* s, const Search::Options& o);
//    /// Whether engine does best solution search
//    static const bool best = E<T>::best;
//  };
//
//  /**
//   * \brief Perform restart-based search
//   *
//   * The engine uses the Cutoff sequence supplied in the options \a o to
//   * periodically restart the search of engine \a E.
//   *
//   * The class \a T can implement member functions
//   * \code virtual bool master(const MetaInfo& mi) \endcode
//   * and
//   * \code virtual bool slave(const MetaInfo& mi) \endcode
//   *
//   * Whenever exploration restarts or a solution is found, the
//   * engine executes the functions on the master and slave
//   * space. For more details, consult "Modeling and Programming
//   * with Gecode".
//   *
//   * \ingroup TaskModelSearch
//   */
//  template<class T, template<class> class E>
//  T* templ_rbs(T* s, const Search::Options& o);
//
//  /// Return a restart search engine builder
//  template<class T, template<class> class E>
//  SEB templ_rbs(const Search::Options& o);
//
//}
//
//#include "search/rbs.hpp"

#endif // TEMPL_SOLVERS_CSP_SEARCH_HPP

