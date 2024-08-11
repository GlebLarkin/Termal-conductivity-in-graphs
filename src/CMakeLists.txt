set(SOURCES
  AnalyticalSolver.cc 
  Controller.cc 
  Graph.cc 
  HeatTransferFacade.cc 
  NumericalSolver.cc 
  Solver.cc 
  main.cc
)

target_sources(TermalConductivityInGraphs PRIVATE ${SOURCES})

