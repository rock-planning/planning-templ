find_package(Boost REQUIRED system filesystem)

rock_library(templ
    SOURCES 
        solvers/temporal/point_algebra/TimePoint.cpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.cpp
        solvers/temporal/Bounds.cpp
        solvers/temporal/SimpleTemporalNetwork.cpp
        solvers/ConstraintNetwork.cpp
        solvers/Constraint.cpp
        solvers/Variable.cpp
        Event.cpp
        PersistenceCondition.cpp
        TemporalAssertion.cpp
    HEADERS 
        solvers/temporal/point_algebra/QualitativeTimePoint.hpp
        solvers/temporal/point_algebra/ExactTimePoint.hpp
        solvers/temporal/point_algebra/TimePoint.hpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp
        solvers/temporal/Bounds.hpp
        solvers/temporal/SimpleTemporalNetwork.hpp
        solvers/ConstraintNetwork.hpp
        solvers/Variable.hpp
        solvers/Constraint.hpp
        Chronicle.hpp
        Event.hpp
        PersistenceCondition.hpp
        StateVariable.hpp
        TemporalAssertion.hpp
        Timeline.hpp
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

