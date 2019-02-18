# @author Munzir Zafar
# @date Dec 12, 2018

cmake_minimum_required(VERSION 2.6)

install(DIRECTORY "3DOF-WIP" "7DOFArm" "Krang" "KrangFixedWheels" "KrangWaist" "scenes"
  DESTINATION /usr/local/share/krang/urdf
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.dae"
  PATTERN "build" EXCLUDE
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


