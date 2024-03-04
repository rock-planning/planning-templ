find_package(Boost REQUIRED system regex filesystem serialization program_options)
rock_testsuite(templ-test suite.cpp
    #test_Scheduling.cpp
    solvers/test_AgentRoutingProblem.cpp
    solvers/test_CSP.cpp
    solvers/test_FluentTimeResource.cpp
    solvers/test_SolutionAnalysis.cpp
    solvers/test_Propagators_IsPath.cpp
    solvers/test_Propagators_IsValidTransportEdge.cpp
    solvers/test_TransportNetwork.cpp
    solvers/test_TemporallyExpandedGraph.cpp
    solvers/test_MissionConstraints.cpp
    test_Constraints.cpp
    test_Benchmarks.cpp
    test_CartographicMapping.cpp
    test_IO.cpp
    test_IntervalConstraint.cpp
    test_LoosePathConsistency.cpp
    test_Mission.cpp
    test_Planning.cpp
    test_PointAlgebra.cpp
    test_RoleInfo.cpp
    test_QualitativeTemporalConstraintNetwork.cpp
    test_SimpleTemporalNetwork.cpp
    test_Solution.cpp
    test_Symbols.cpp
    test_TemporalAssertion.cpp
    test_TemporalConstraintNetwork.cpp
    test_TimePoint.cpp
    test_Timeline.cpp
    test_Utils.cpp
    DEPS
        templ
        templ_benchmark
    LIBS ${Boost_LIBRARIES}
    )
