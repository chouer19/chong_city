set (dirs
  actor
  city_terrain
  ocean
  my_prius
)

INSTALL(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim-${CITYSIM_MAJOR_VERSION}/models)
