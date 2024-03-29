find_package(Boost REQUIRED system regex filesystem serialization program_options)
find_package(gecode REQUIRED)

# For adding the qt4 resources
set(CMAKE_AUTORCC ON)
rock_library(templ_core
    SOURCES
        Constraint.cpp
        ConstraintNetwork.cpp
        Role.cpp
        RoleInfo.cpp
        RoleInfoWeightedEdge.cpp
        Symbol.cpp
        Variable.cpp
        constraints/HyperConstraint.cpp
        constraints/SimpleConstraint.cpp
        io/FluentTypes.cpp
        problems/Scheduling.cpp
        solvers/Cost.cpp
        symbols/Constant.cpp
        symbols/ObjectVariable.cpp
        symbols/StateVariable.cpp
        symbols/Value.cpp
        symbols/constants/Location.cpp
        symbols/object_variables/LocationCardinality.cpp
        symbols/object_variables/LocationNumericAttribute.cpp
        utils/CSVLogger.cpp
        utils/CartographicMapping.cpp
        utils/Logger.cpp
    HEADERS
        Constraint.hpp
        ConstraintNetwork.hpp
        Role.hpp
        RoleInfo.hpp
        RoleInfoTuple.hpp
        RoleInfoWeightedEdge.hpp
        Symbol.hpp
        Tuple.hpp
        Variable.hpp
        constraints/HyperConstraint.hpp
        constraints/SimpleConstraint.hpp
        io/FluentTypes.hpp
        solvers/Cost.hpp
        symbols/Constant.hpp
        symbols/ObjectVariable.hpp
        symbols/StateVariable.hpp
        symbols/Value.hpp
        symbols/constants/Location.hpp
        symbols/object_variables/LocationCardinality.hpp
        symbols/object_variables/LocationNumericAttribute.hpp
        symbols/values/Int.hpp
        utils/CSVLogger.hpp
        utils/CartographicMapping.hpp
        utils/Logger.hpp
    LIBS ${Boost_LIBRARIES}
        proj
        gecode::kernel
        gecode::driver
        gecode::set
        gecode::support
        gecode::search 
        gecode::int 
        gecode::float
        gecode::minimodel
        gecode::flatzinc
        gecode::gist
    DEPS_PKGCONFIG graph_analysis
        numeric
        owlapi
        moreorg
        libxml-2.0
        base-types
        eigen3
        qxcfg
)

if(WITH_GQR)
    set(MOFT_EXTRA_CPP
        solvers/GQReasoner.cpp
    )

    set(MOFT_EXTRA_HPP
        solvers/GQReasoner.hpp
    )
    list(APPEND templ_moft_DEPS_PKGCONFIG gqr)
    list(APPEND PKGCONFIG_MOFT_REQUIRES gqr)
endif(WITH_GQR)

# Master of time
rock_library(templ_moft
    SOURCES
        RoleInfoTuple.cpp
        SpaceTime.cpp
        SpaceTimeNetwork.cpp
        io/TemporalConstraint.cpp
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
        utils/XMLTCNUtils.cpp
        ${MOFT_EXTRA_CPP}
    HEADERS
        SpaceTime.hpp
        SpaceTimeNetwork.hpp
        TemporallyExpandedNetwork.hpp
        io/TemporalConstraint.hpp
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
        utils/XMLTCNUtils.hpp
        ${MOFT_EXTRA_HPP}
    DEPS
        templ_core
    LIBS ${Boost_LIBRARIES}
)
if(WITH_GQR)
    target_link_libraries(templ_moft ${GQR_LIBRARIES})
endif(WITH_GQR)

rock_library(templ
    SOURCES
        CapacityLink.cpp
        DataPropertyAssignment.cpp
        Mission.cpp
        constraints/ModelConstraint.cpp
        Plan.cpp
        solvers/Solver.cpp
        solvers/Session.cpp
        solvers/FluentTimeResource.cpp
        io/LatexWriter.cpp
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
        solvers/csp/Context.cpp
        solvers/csp/FlawResolution.cpp
        solvers/csp/MissionConstraints.cpp
        solvers/csp/MissionConstraintManager.cpp
        solvers/csp/RoleTimeline.cpp
        solvers/csp/TransportNetwork.cpp
        solvers/csp/Types.cpp
        solvers/csp/branchers/SetNGL.cpp
        solvers/csp/branchers/TimelineBrancher.cpp
        solvers/csp/propagators/Idx.cpp
        solvers/csp/propagators/InEdgesRestriction.cpp
        solvers/csp/propagators/IsPath.cpp
        solvers/csp/propagators/IsValidTransportEdge.cpp
        solvers/csp/propagators/MultiCommodityFlow.cpp
        #solvers/csp/search/rbs.cpp
        #solvers/csp/search/meta/rbs.cpp
        solvers/csp/utils/Converter.cpp
        solvers/csp/utils/FluentTimeIndex.cpp
        solvers/csp/utils/Formatter.cpp
        solvers/Solution.cpp
        solvers/SolutionAnalysis.cpp
        solvers/transshipment/Flaw.cpp
        solvers/transshipment/MinCostFlow.cpp
        solvers/transshipment/FlowNetwork.cpp
        utils/PathConstructor.cpp
    HEADERS
        CapacityLink.hpp
        DataPropertyAssignment.hpp
        Mission.hpp
        constraints/ModelConstraint.hpp
        Plan.hpp
        SharedPtr.hpp
        io/LatexWriter.hpp
        io/MissionReader.hpp
        io/MissionWriter.hpp
        io/MissionRequirements.hpp
        solvers/Solver.hpp
        solvers/Session.hpp
        solvers/FluentTimeResource.hpp
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
        solvers/csp/Context.hpp
        solvers/csp/FlawResolution.hpp
        solvers/csp/MissionConstraints.hpp
        solvers/csp/MissionConstraintManager.hpp
        solvers/csp/RoleTimeline.hpp
        solvers/csp/TransportNetwork.hpp
        solvers/csp/Types.hpp
        solvers/csp/branchers/SetNGL.hpp
        solvers/csp/branchers/TimelineBrancher.hpp
        solvers/csp/propagators/Idx.hpp
        solvers/csp/propagators/InEdgesRestriction.hpp
        solvers/csp/propagators/IsPath.hpp
        solvers/csp/propagators/IsValidTransportEdge.hpp
        solvers/csp/propagators/MultiCommodityFlow.hpp
        #solvers/csp/Search.hpp
        #solvers/csp/search/rbs.hpp
        #solvers/csp/search/meta/rbs.hh
        #solvers/csp/search/meta/rbs.hpp
        solvers/csp/utils/Converter.hpp
        solvers/csp/utils/FluentTimeIndex.hpp
        solvers/csp/utils/Formatter.hpp
        solvers/transshipment/Flaw.hpp
        solvers/transshipment/MinCostFlow.hpp
        solvers/transshipment/FlowNetwork.hpp
        solvers/Solution.hpp
        solvers/SolutionAnalysis.hpp
        utils/PathConstructor.hpp
    DEPS
        templ_moft
    LIBS ${Boost_LIBRARIES}
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
    LIBS ${Boost_LIBRARIES}
)

rock_executable(templ_bin Main.cpp
    DEPS templ)

rock_executable(templ-mission_reader
    utils/MissionReaderMain.cpp
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

rock_executable(templ-narrow_mission
    utils/NarrowMission.cpp
    DEPS templ
)

rock_executable(templ-solver
    utils/SolverMain.cpp
    DEPS templ
)

rock_executable(templ-mission_generator
    utils/MissionGeneratorMain.cpp
    DEPS templ_benchmark
)

rock_executable(templ-bm-tcn utils/TCNBenchmark.cpp
    DEPS templ)

rock_library(templ_gui
    HEADERS
        gui/TemplGui.hpp
        gui/Utils.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/MouseWheelGuard.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
        gui/dialogs/AddLocation.hpp
        gui/dialogs/AddModelConstraint.hpp
        gui/dialogs/AddTemporalConstraintQualitative.hpp
        gui/dialogs/AddTemporalConstraintQuantitative.hpp
        gui/edge_items/RoleInfoItem.hpp
        gui/edge_items/CapacityLinkItem.hpp
        gui/vertex_items/RoleInfoItem.hpp
        gui/models/AgentStyleModel.hpp
        gui/solution_analysis/SolutionProperties.hpp
        gui/widgets/Location.hpp
        gui/widgets/ModelCardinality.hpp
        gui/widgets/ModelConstraint.hpp
        gui/widgets/PenStyle.hpp
        gui/widgets/PropertyOverride.hpp
        gui/widgets/SpatioTemporalRequirement.hpp
        gui/widgets/TemporalConstraintQualitative.hpp
        gui/widgets/TemporalConstraintQuantitative.hpp
    SOURCES
        gui/TemplGui.cpp
        gui/Utils.cpp
        gui/SpatioTemporalRequirementItem.cpp
        gui/MissionEditor/MissionEditor.cpp
        gui/MissionView/MissionView.cpp
        gui/MouseWheelGuard.cpp
        gui/OntologyView/OntologyView.cpp
        gui/OntologyView/OntologyTreeModel.cpp
        gui/dialogs/AddLocation.cpp
        gui/dialogs/AddModelConstraint.cpp
        gui/dialogs/AddTemporalConstraintQualitative.cpp
        gui/dialogs/AddTemporalConstraintQuantitative.cpp
        gui/edge_items/RoleInfoItem.cpp
        gui/edge_items/CapacityLinkItem.cpp
        gui/solution_analysis/SolutionProperties.cpp
        gui/vertex_items/RoleInfoItem.cpp
        gui/models/AgentStyleModel.cpp
        gui/widgets/Location.cpp
        gui/widgets/ModelCardinality.cpp
        gui/widgets/ModelConstraint.cpp
        gui/widgets/PenStyle.cpp
        gui/widgets/PropertyOverride.cpp
        gui/widgets/SpatioTemporalRequirement.cpp
        gui/widgets/TemporalConstraintQualitative.cpp
        gui/widgets/TemporalConstraintQuantitative.cpp
        ../resources.qrc
    MOC
        gui/TemplGui.hpp
        gui/SpatioTemporalRequirementItem.hpp
        gui/MissionEditor/MissionEditor.hpp
        gui/MissionView/MissionView.hpp
        gui/OntologyView/OntologyView.hpp
        gui/OntologyView/OntologyTreeModel.hpp
        gui/dialogs/AddLocation.hpp
        gui/dialogs/AddModelConstraint.hpp
        gui/dialogs/AddTemporalConstraintQualitative.hpp
        gui/dialogs/AddTemporalConstraintQuantitative.hpp
        gui/edge_items/RoleInfoItem.hpp
        gui/edge_items/CapacityLinkItem.hpp
        gui/vertex_items/RoleInfoItem.hpp
        gui/models/AgentStyleModel.hpp
        gui/solution_analysis/SolutionProperties.hpp
        gui/widgets/Location.hpp
        gui/widgets/ModelCardinality.hpp
        gui/widgets/ModelConstraint.hpp
        gui/widgets/PenStyle.hpp
        gui/widgets/PropertyOverride.hpp
        gui/widgets/SpatioTemporalRequirement.hpp
        gui/widgets/TemporalConstraintQualitative.hpp
        gui/widgets/TemporalConstraintQuantitative.hpp
    DEPS
        templ
    DEPS_PKGCONFIG
        QtGui
        QtXml
    UI
        gui/TemplGui.ui
        gui/dialogs/AddLocation.ui
        gui/dialogs/EmptyDialog.ui
        gui/MissionEditor/MissionEditor.ui
        gui/OntologyView/OntologyView.ui
        gui/solution_analysis/SolutionProperties.ui
        gui/widgets/Location.ui
        gui/widgets/ModelCardinality.ui
        gui/widgets/ModelConstraint.ui
        gui/widgets/PenStyle.ui
        gui/widgets/PropertyOverride.ui
        gui/widgets/SpatioTemporalRequirement.ui
        gui/widgets/TemporalConstraintQualitative.ui
        gui/widgets/TemporalConstraintQuantitative.ui
    LIBS ${Boost_LIBRARIES}
)

rock_executable(templ-gui gui/Main.cpp
    DEPS templ_gui)

