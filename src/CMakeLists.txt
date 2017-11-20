find_package(Boost REQUIRED system filesystem serialization program_options)

rock_library(templ_core
    SOURCES
        Role.cpp
        RoleInfo.cpp
        RoleInfoWeightedEdge.cpp
        Symbol.cpp
        io/FluentTypes.cpp
        problems/Scheduling.cpp
        solvers/Constraint.cpp
        solvers/Cost.cpp
        solvers/ConstraintNetwork.cpp
        solvers/Variable.cpp
        symbols/Constant.cpp
        symbols/ObjectVariable.cpp
        symbols/StateVariable.cpp
        symbols/Value.cpp
        symbols/constants/Location.cpp
        symbols/object_variables/LocationCardinality.cpp
        symbols/object_variables/LocationNumericAttribute.cpp
        utils/CartographicMapping.cpp
    HEADERS
        Role.hpp
        RoleInfo.hpp
        RoleInfoTuple.hpp
        RoleInfoWeightedEdge.hpp
        Symbol.hpp
        Tuple.hpp
        Variable.hpp
        io/FluentTypes.hpp
        solvers/Constraint.hpp
        solvers/ConstraintNetwork.hpp
        solvers/Cost.hpp
        solvers/Variable.hpp
        symbols/Constant.hpp
        symbols/ObjectVariable.hpp
        symbols/StateVariable.hpp
        symbols/Value.hpp
        symbols/constants/Location.hpp
        symbols/object_variables/LocationCardinality.hpp
        symbols/object_variables/LocationNumericAttribute.hpp
        symbols/values/Int.hpp
        utils/CartographicMapping.hpp
    LIBS ${Boost_LIBRARIES}
        proj
    DEPS_PKGCONFIG graph_analysis
        numeric
        owlapi
        organization_model
        libxml-2.0
        base-types
        eigen3
)

rock_library(templ_moft
    SOURCES
        solvers/GQReasoner.cpp
        solvers/csp/TemporalConstraintNetwork.cpp
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
    HEADERS
        solvers/GQReasoner.hpp
        solvers/csp/TemporalConstraintNetwork.hpp
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
    DEPS
        templ_core
    DEPS_PKGCONFIG
        gqr
)

rock_library(templ
    SOURCES
        CapacityLink.cpp
        Configuration.cpp
        DataPropertyAssignment.cpp
        Mission.cpp
        MissionPlanner.cpp
        Plan.cpp
        PlanSpaceVisitor.cpp
        PlanningState.cpp
        RoleInfoTuple.cpp
        SpaceTime.cpp
        io/MissionReader.cpp
        io/MissionWriter.cpp
        io/MissionRequirements.cpp
        solvers/agent_routing/Agent.cpp
        solvers/agent_routing/AgentIntegerAttribute.cpp
        solvers/agent_routing/AgentRoutingGraph.cpp
        solvers/agent_routing/AgentRoutingProblem.cpp
        solvers/agent_routing/AgentTask.cpp
        solvers/agent_routing/AgentType.cpp
        solvers/agent_routing/ArpIO.cpp
        solvers/agent_routing/io/XMLReader.cpp
        solvers/agent_routing/io/XMLWriter.cpp
        solvers/csp/ConstraintMatrix.cpp
        solvers/csp/FluentTimeResource.cpp
        solvers/csp/FlawResolution.cpp
        solvers/csp/MissionConstraints.cpp
        solvers/csp/ModelDistribution.cpp
        solvers/csp/Resolver.cpp
        solvers/csp/RoleDistribution.cpp
        solvers/csp/RoleTimeline.cpp
        solvers/csp/TemporalCSP.cpp
        solvers/csp/TransportNetwork.cpp
        solvers/csp/Types.cpp
        solvers/csp/branchers/TimelineBrancher.cpp
        solvers/csp/branchers/SetNGL.cpp
        solvers/csp/propagators/IsPath.cpp
        solvers/csp/propagators/IsValidTransportEdge.cpp
        solvers/csp/propagators/MultiCommodityFlow.cpp
        solvers/csp/search/rbs.cpp
        solvers/csp/search/meta/rbs.cpp
        solvers/csp/utils/Converter.cpp
        solvers/csp/utils/FluentTimeIndex.cpp
        solvers/csp/utils/Formatter.cpp
        solvers/transshipment/Flaw.cpp
        solvers/transshipment/MinCostFlow.cpp
        solvers/transshipment/FlowNetwork.cpp
        solvers/SolutionAnalysis.cpp
        utils/CSVLogger.cpp
        utils/Logger.cpp
        utils/PathConstructor.cpp
        utils/XMLUtils.cpp
    HEADERS
        CapacityLink.hpp
        Configuration.hpp
        DataPropertyAssignment.hpp
        Mission.hpp
        MissionPlanner.hpp
        Plan.hpp
        PlanSpaceVisitor.hpp
        PlanningState.hpp
        SpaceTime.hpp
        TemporallyExpandedNetwork.hpp
        io/MissionReader.hpp
        io/MissionWriter.hpp
        io/MissionRequirements.hpp
        solvers/agent_routing/Agent.hpp
        solvers/agent_routing/AgentIntegerAttribute.hpp
        solvers/agent_routing/AgentRoutingGraph.hpp
        solvers/agent_routing/AgentRoutingProblem.hpp
        solvers/agent_routing/AgentTask.hpp
        solvers/agent_routing/AgentType.hpp
        solvers/agent_routing/ArpIO.hpp
        solvers/agent_routing/io/XMLReader.hpp
        solvers/agent_routing/io/XMLWriter.hpp
        solvers/csp/ConstraintMatrix.hpp
        solvers/csp/FluentTimeResource.hpp
        solvers/csp/FlawResolution.hpp
        solvers/csp/MissionConstraints.hpp
        solvers/csp/ModelDistribution.hpp
        solvers/csp/Resolver.hpp
        solvers/csp/RoleDistribution.hpp
        solvers/csp/RoleTimeline.hpp
        solvers/csp/TemporalCSP.hpp
        solvers/csp/TransportNetwork.hpp
        solvers/csp/Types.hpp
        solvers/csp/branchers/TimelineBrancher.hpp
        solvers/csp/branchers/SetNGL.hpp
        solvers/csp/propagators/IsPath.hpp
        solvers/csp/propagators/IsValidTransportEdge.hpp
        solvers/csp/propagators/MultiCommodityFlow.hpp
        solvers/csp/Search.hpp
        solvers/csp/search/rbs.hpp
        solvers/csp/search/meta/rbs.hh
        solvers/csp/search/meta/rbs.hpp
        solvers/csp/utils/Converter.hpp
        solvers/csp/utils/FluentTimeIndex.hpp
        solvers/csp/utils/Formatter.hpp
        solvers/transshipment/Flaw.hpp
        solvers/transshipment/MinCostFlow.hpp
        solvers/transshipment/FlowNetwork.hpp
        solvers/SolutionAnalysis.hpp
        utils/PathConstructor.hpp
        utils/CSVLogger.hpp
        utils/Logger.hpp
        utils/XMLUtils.hpp
    DEPS
        templ_moft
)

rock_library(templ_benchmark
    SOURCES
        benchmark/MissionGenerator.cpp
        benchmark/VRPProblem.cpp
        benchmark/io/GoldenReader.cpp
    HEADERS
        benchmark/MissionGenerator.hpp
        benchmark/VRPProblem.hpp
        benchmark/io/GoldenReader.hpp
    DEPS
        templ
)


set(GECODE_LIBRARIES
        gecodeflatzinc
        gecodedriver
        gecodegist
        gecodesearch
        gecodeminimodel
        gecodeset
        gecodefloat
        gecodeint
        gecodekernel
        gecodesupport
)

message(STATUS ${GECODE_LIBRARIES})

foreach(GECODE_LIB ${GECODE_LIBRARIES})
    find_library(${GECODE_LIB}_PATH NAMES ${GECODE_LIB} PATHS ${CMAKE_INSTALL_TARGET}/lib
        REQUIRED )
    message(STATUS "LINK ${${GECODE_LIB}_PATH}")
    target_link_libraries(templ ${${GECODE_LIB}_PATH})
endforeach()

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
    DEPS templ templ_core
)

rock_executable(templ-solution_analysis
    utils/SolutionAnalyserMain.cpp
    DEPS templ
)

rock_executable(templ-mission_generator
    utils/MissionGeneratorMain.cpp
    DEPS templ_benchmark
)

rock_library(templ_gui
    HEADERS
        gui/TemplGui.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
        gui/edge_items/RoleInfoItem.hpp
        gui/edge_items/CapacityLinkItem.hpp
        gui/vertex_items/RoleInfoItem.hpp
    SOURCES
        gui/TemplGui.cpp
        gui/SpatioTemporalRequirementItem.cpp
        gui/MissionEditor/MissionEditor.cpp
        gui/MissionView/MissionView.cpp
        gui/OntologyView/OntologyView.cpp
        gui/OntologyView/OntologyTreeModel.cpp
        gui/edge_items/RoleInfoItem.cpp
        gui/edge_items/CapacityLinkItem.cpp
        gui/vertex_items/RoleInfoItem.cpp
    MOC
        gui/TemplGui.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
        gui/edge_items/RoleInfoItem.hpp
        gui/edge_items/CapacityLinkItem.hpp
        gui/vertex_items/RoleInfoItem.hpp
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

