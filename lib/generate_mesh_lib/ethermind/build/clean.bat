@echo off

REM Root Setting
SET ROOT=..

SET target=phy6200
SET target_core=arm_cm0
SET target_os=phyos
SET tool_chain=keil

make clean -f %ROOT%/mesh/build/make/Makefile-ethermind-mesh-core TOOL_CHAIN=%tool_chain% TARGET=%target% TARGET_CORE=%target_core% TARGET_OS=%target_os%
make clean -f %ROOT%/utils/build/make/Makefile-ethermind-utils TOOL_CHAIN=%tool_chain% TARGET=%target% TARGET_CORE=%target_core% TARGET_OS=%target_os%

