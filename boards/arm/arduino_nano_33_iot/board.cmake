# Copyright (c) 2020 Google LLC.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=atsamd21g18a" "--frequency=4000000")
include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
