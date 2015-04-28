find_package(Boost REQUIRED system filesystem)

rock_library(templ
    SOURCES 
        ObjectVariable.cpp
        solvers/temporal/Bounds.cpp
        solvers/temporal/Chronicle.cpp
        solvers/temporal/Event.cpp
        solvers/temporal/PersistenceCondition.cpp
        solvers/temporal/point_algebra/QualitativeTimePoint.cpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.cpp
        solvers/temporal/point_algebra/TimePoint.cpp
        solvers/temporal/point_algebra/TimePointComparator.cpp
        solvers/temporal/QualitativeTemporalConstraintNetwork.cpp
        solvers/temporal/SimpleTemporalNetwork.cpp
        solvers/temporal/TemporalAssertion.cpp
        solvers/temporal/TemporalConstraintNetwork.cpp
        solvers/temporal/Timeline.cpp
        solvers/ConstraintNetwork.cpp
        solvers/Constraint.cpp
        solvers/Variable.cpp
        PlannerElement.cpp
        Value.cpp
    HEADERS 
        Constant.hpp
        ObjectVariable.hpp
        PlannerElement.hpp
        solvers/temporal/Bounds.hpp
        solvers/temporal/Chronicle.hpp
        solvers/temporal/Event.hpp
        solvers/temporal/point_algebra/ExactTimePoint.hpp
        solvers/temporal/point_algebra/QualitativeTimePoint.hpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp
        solvers/temporal/point_algebra/TimePoint.hpp
        solvers/temporal/point_algebra/TimePointComparator.hpp
        solvers/temporal/PersistenceCondition.hpp
        solvers/temporal/QualitativeTemporalConstraintNetwork.hpp
        solvers/temporal/SimpleTemporalNetwork.hpp
        solvers/temporal/TemporalAssertion.hpp
        solvers/temporal/TemporalConstraintNetwork.hpp
        solvers/temporal/Timeline.hpp
        solvers/ConstraintNetwork.hpp
        solvers/Variable.hpp
        solvers/Constraint.hpp
        StateVariable.hpp
        Value.hpp
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

