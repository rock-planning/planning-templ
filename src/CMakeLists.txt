find_package(Boost REQUIRED system filesystem)

rock_library(templ
    SOURCES 
        Mission.cpp
        MissionPlanner.cpp
        ObjectVariable.cpp
        solvers/csp/ModelDistribution.cpp
        solvers/temporal/Bounds.cpp
        solvers/temporal/Chronicle.cpp
        solvers/temporal/Event.cpp
        solvers/temporal/Interval.cpp
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
        Mission.hpp
        MissionPlanner.hpp
        ObjectVariable.hpp
        PlannerElement.hpp
        solvers/csp/ModelDistribution.hpp
        solvers/temporal/Bounds.hpp
        solvers/temporal/Chronicle.hpp
        solvers/temporal/Event.hpp
        solvers/temporal/Interval.hpp
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
    LIBS gecodesearch
        gecodesupport
        gecodeset
        gecodekernel
        gecodeminimodel
        gecodeint
        gecodefloat
        gecodegist
        ${Boost_LIBRARIES}
    DEPS_PKGCONFIG graph_analysis numeric organization_model
)

rock_executable(templ_bin Main.cpp
    DEPS templ)

