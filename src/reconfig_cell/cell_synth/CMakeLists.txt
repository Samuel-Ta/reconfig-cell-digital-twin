cmake_minimum_required(VERSION 3.8)
project(cell_synth)
find_package(ament_cmake REQUIRED)

# Rung 1 synthesis: the oracle/proposer library + the two entry scripts.
# synth_core.py is installed alongside the scripts (same lib/ dir) so they import it.
install(PROGRAMS
  scripts/synth_core.py
  scripts/oracle_smoke
  scripts/synthesize
  scripts/score_probe
  scripts/optimize
  scripts/valset
  scripts/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
