find_package(Boost REQUIRED system filesystem)

rock_library(templ
    SOURCES
        CapacityLink.cpp
        Logger.cpp
        Mission.cpp
        MissionPlanner.cpp
        PathConstructor.cpp
        Plan.cpp
        PlanningState.cpp
        PlanSpaceVisitor.cpp
        Role.cpp
        Symbol.cpp
        io/MissionReader.cpp
        problems/Scheduling.cpp
        solvers/Constraint.cpp
        solvers/ConstraintNetwork.cpp
        solvers/Variable.cpp
        solvers/csp/ConstraintMatrix.cpp
        solvers/csp/ModelDistribution.cpp
        solvers/csp/Resolver.cpp
        solvers/csp/RoleDistribution.cpp
        solvers/csp/RoleTimeline.cpp
        solvers/csp/FluentTimeResource.cpp
        solvers/csp/TemporalCSP.cpp
        solvers/csp/TransportNetwork.cpp
        solvers/csp/propagators/IsPath.cpp
        solvers/csp/propagators/MultiCommodityFlow.cpp
        solvers/csp/utils/FluentTimeIndex.cpp
        solvers/csp/utils/Formatter.cpp
        solvers/GQReasoner.cpp
        solvers/temporal/Bounds.cpp
        solvers/temporal/Chronicle.cpp
        solvers/temporal/Event.cpp
        solvers/temporal/Interval.cpp
        solvers/temporal/IntervalConstraint.cpp
        solvers/temporal/LoosePathConsistency.cpp
        solvers/temporal/PersistenceCondition.cpp
        solvers/temporal/QualitativeTemporalConstraintNetwork.cpp
        solvers/temporal/SimpleTemporalNetwork.cpp
        solvers/temporal/TemporalAssertion.cpp
        solvers/temporal/TemporalConstraintNetwork.cpp
        solvers/temporal/Timeline.cpp
        solvers/temporal/point_algebra/QualitativeTimePoint.cpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.cpp
        solvers/temporal/point_algebra/TimePoint.cpp
        solvers/temporal/point_algebra/TimePointComparator.cpp
        solvers/transshipment/MinCostFlow.cpp
        solvers/transshipment/TransportNetwork.cpp
        symbols/Constant.cpp
        symbols/ObjectVariable.cpp
        symbols/StateVariable.cpp
        symbols/Value.cpp
        symbols/object_variables/LocationCardinality.cpp
        utils/CartographicMapping.cpp
    HEADERS
        CapacityLink.hpp
        Logger.hpp
        Mission.hpp
        MissionPlanner.hpp
        PathConstructor.hpp
        Plan.hpp
        PlanningState.hpp
        PlanSpaceVisitor.hpp
        Role.hpp
        RoleInfoTuple.hpp
        SpaceTimeNetwork.hpp
        Symbol.hpp
        TemporallyExpandedNetwork.hpp
        Tuple.hpp
        Variable.hpp
        io/MissionReader.hpp
        solvers/Constraint.hpp
        solvers/ConstraintNetwork.hpp
        solvers/GQReasoner.hpp
        solvers/Variable.hpp
        solvers/csp/ConstraintMatrix.hpp
        solvers/csp/FluentTimeResource.hpp
        solvers/csp/ModelDistribution.hpp
        solvers/csp/Resolver.hpp
        solvers/csp/RoleDistribution.hpp
        solvers/csp/RoleTimeline.hpp
        solvers/csp/TemporalCSP.hpp
        solvers/csp/TransportNetwork.hpp
        solvers/csp/propagators/IsPath.hpp
        solvers/csp/propagators/MultiCommodityFlow.hpp
        solvers/csp/utils/FluentTimeIndex.hpp
        solvers/csp/utils/Formatter.hpp
        solvers/temporal/Bounds.hpp
        solvers/temporal/Chronicle.hpp
        solvers/temporal/Event.hpp
        solvers/temporal/Interval.hpp
        solvers/temporal/IntervalConstraint.hpp
        solvers/temporal/LoosePathConsistency.hpp
        solvers/temporal/PersistenceCondition.hpp
        solvers/temporal/QualitativeTemporalConstraintNetwork.hpp
        solvers/temporal/SimpleTemporalNetwork.hpp
        solvers/temporal/TemporalAssertion.hpp
        solvers/temporal/TemporalConstraintNetwork.hpp
        solvers/temporal/Timeline.hpp
        solvers/temporal/point_algebra/ExactTimePoint.hpp
        solvers/temporal/point_algebra/QualitativeTimePoint.hpp
        solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp
        solvers/temporal/point_algebra/TimePoint.hpp
        solvers/temporal/point_algebra/TimePointComparator.hpp
        solvers/transshipment/MinCostFlow.hpp
        solvers/transshipment/TransportNetwork.hpp
        symbols/Constant.hpp
        symbols/ObjectVariable.hpp
        symbols/StateVariable.hpp
        symbols/Value.hpp
        symbols/constants/Location.hpp
        symbols/object_variables/LocationCardinality.hpp
        symbols/values/Int.hpp
        utils/CartographicMapping.hpp
    LIBS gecodesearch
        gecodesupport
        gecodeset
        gecodekernel
        gecodeminimodel
        gecodeint
        gecodefloat
        gecodegist
        ${Boost_LIBRARIES}
        proj
    DEPS_PKGCONFIG graph_analysis numeric organization_model
        libxml-2.0 base-types gqr eigen3
)

rock_executable(templ_bin Main.cpp
    DEPS templ)

rock_executable(templ-mission_reader
    utils/MissionReaderMain.cpp
    DEPS templ
)

rock_executable(templ-mission_planner
    utils/MissionPlannerMain.cpp
    DEPS templ
)

rock_executable(templ-transport_network_planner
    utils/TransportNetworkPlannerMain.cpp
    DEPS templ
)

rock_executable(templ-tcn_reader
    utils/TemporalConstraintNetworkMain.cpp
    DEPS templ
)

rock_library(templ_gui
    HEADERS
        gui/TemplGui.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
    SOURCES
        gui/TemplGui.cpp
        gui/SpatioTemporalRequirementItem.cpp
        gui/MissionEditor/MissionEditor.cpp
        gui/MissionView/MissionView.cpp
        gui/OntologyView/OntologyView.cpp
        gui/OntologyView/OntologyTreeModel.cpp
    MOC
        gui/TemplGui.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
    DEPS
        templ
    DEPS_PKGCONFIG
        QtGui
    UI
        gui/TemplGui.ui
        gui/MissionEditor/MissionEditor.ui
        gui/OntologyView/OntologyView.ui
)

rock_executable(templ-gui gui/Main.cpp
    DEPS templ_gui)

