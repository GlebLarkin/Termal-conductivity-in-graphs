include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(HEADERS
    AnalyticalSolver.hpp
    Controller.hpp
    Graph.hpp
    HeatTransferFacade.hpp
    MyInclude.hpp
    NumericalSolver.hpp
    Solver.hpp
)

set_property(TARGET TermalConductivityInGraphs PROPERTY INCLUDE_DIRECTORIES ${HEADERS})

