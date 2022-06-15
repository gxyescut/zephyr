# Copyright (c) 2019 SiFive Inc.
# SPDX-License-Identifier: Apache-2.0

set(SUPPORTED_EMU_PLATFORMS renode)
set(RENODE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/support/hifive1_revb.resc)

board_runner_args(jlink "--device=FE310")
board_runner_args(jlink "--iface=JTAG")
board_runner_args(jlink "--speed=4000")
board_runner_args(jlink "--tool-opt=-jtagconf -1,-1")
board_runner_args(jlink "--tool-opt=-autoconnect 1")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
