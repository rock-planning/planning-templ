rock_testsuite(test_suite suite.cpp
    #test_Scheduling.cpp
    solvers/test_CSP.cpp
    solvers/test_Propagators.cpp
    solvers/test_TransportNetwork.cpp
    solvers/test_TemporalCSP.cpp
    test_CartographicMapping.cpp
    test_LoosePathConsistency.cpp
    test_Mission.cpp
    test_Planning.cpp
    test_PointAlgebra.cpp
    test_QualitativeTemporalConstraintNetwork.cpp
    test_SimpleTemporalNetwork.cpp
    test_Symbols.cpp
    test_TemporalAssertion.cpp
    test_TemporalConstraintNetwork.cpp
    test_TimePoint.cpp
    test_Timeline.cpp
    DEPS templ)
