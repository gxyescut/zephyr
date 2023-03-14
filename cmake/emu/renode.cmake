# SPDX-License-Identifier: Apache-2.0

find_program(
  RENODE
  renode
  )

set(RENODE_FLAGS
  --disable-xwt
  --port -2
  --pid-file renode.pid
  )

set(RENODE_COMMAND
  -e '$$bin=@${APPLICATION_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}\\\; include @${RENODE_SCRIPT}\\\; s'
  )

# Check if there is any Renode script overlay defined for the target board
set(resc_overlay_file ${APPLICATION_SOURCE_DIR}/boards/${BOARD}.resc)
if(EXISTS ${resc_overlay_file})
  set(RENODE_OVERLAY include "@${resc_overlay_file}\;")
  message(STATUS "Found Renode script overlay: ${resc_overlay_file}")
endif()

add_custom_target(run_renode
  COMMAND
  ${RENODE}
  ${RENODE_FLAGS}
  ${RENODE_COMMAND}
  WORKING_DIRECTORY ${APPLICATION_BINARY_DIR}
  DEPENDS ${logical_target_for_zephyr_elf}
  USES_TERMINAL
  )

#
# these are for running renode-test with west:
#
find_program(
  RENODE_TEST
  renode-test
  )

set(RENODE_TEST_FLAGS
  --variable ELF:@${APPLICATION_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
  --variable RESC:@${RENODE_SCRIPT}
  --variable UART:${RENODE_UART}
  )

set(
  ROBOT_FILES
  *.robot
  )

add_custom_target(run_renode_test
  COMMAND
  ${RENODE_TEST}
  ${RENODE_TEST_FLAGS}
  ${APPLICATION_SOURCE_DIR}/${ROBOT_FILES}
  WORKING_DIRECTORY ${APPLICATION_BINARY_DIR}
  DEPENDS ${logical_target_for_zephyr_elf}
  USES_TERMINAL
  )
