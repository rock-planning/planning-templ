find_package(Boost REQUIRED system filesystem)

rock_library(templ
    SOURCES 
        solvers/temporal/Bounds.cpp
        solvers/temporal/Event.cpp
        solvers/temporal/PersistenceCondition.cpp
        solvers/temporal/point_algebra/TimePoint.cpp
        solvers/temporal/point_algebra/TimePointComparator.cpp
        solvers/temporal/point_algebra/QualitativeTimePoint.cpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.cpp
        solvers/temporal/TemporalAssertion.cpp
        solvers/temporal/SimpleTemporalNetwork.cpp
        solvers/ConstraintNetwork.cpp
        solvers/Constraint.cpp
        solvers/Variable.cpp
    HEADERS 
        solvers/temporal/Bounds.hpp
        solvers/temporal/Chronicle.hpp
        solvers/temporal/Event.hpp
        solvers/temporal/point_algebra/QualitativeTimePoint.hpp
        solvers/temporal/point_algebra/ExactTimePoint.hpp
        solvers/temporal/point_algebra/TimePoint.hpp
        solvers/temporal/point_algebra/TimePointComparator.hpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp
        solvers/temporal/PersistenceCondition.hpp
        solvers/temporal/SimpleTemporalNetwork.hpp
        solvers/temporal/TemporalAssertion.hpp
        solvers/temporal/Timeline.hpp
        solvers/ConstraintNetwork.hpp
        solvers/Variable.hpp
        solvers/Constraint.hpp
        StateVariable.hpp
    DEPS_PKGCONFIG graph_analysis numeric
)

set(GECODE_LIBRARIES
    gecodesearch
    gecodesupport
    gecodeset
    gecodekernel
    gecodeminimodel
    gecodeint
    gecodefloat
    gecodegist
)

target_link_libraries(templ ${Boost_LIBRARIES} ${GECODE_LIBRARIES})

rock_executable(templ_bin Main.cpp
    DEPS templ)

