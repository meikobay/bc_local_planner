FILE(REMOVE_RECURSE
  "CMakeFiles/bc_local_planner_gencfg"
  "devel/include/bc_local_planner/bc_localConfig.h"
  "devel/share/bc_local_planner/docs/bc_localConfig.dox"
  "devel/share/bc_local_planner/docs/bc_localConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/bc_local_planner/cfg/bc_localConfig.py"
  "devel/share/bc_local_planner/docs/bc_localConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/bc_local_planner_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
