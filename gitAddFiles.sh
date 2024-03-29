#! /bin/sh

git add gitAddFiles.bat
git add gitAddFiles.sh
git add checkOriginMaster.bat
git add checkOriginMaster.sh

git add *
git add plasticWelder/*
git add plasticWelder/plastic-welder/*
git add michisLamps/*
git add tempomat/*
git add nixi/*
git add nixi/nixitest/
git add nixi/tubeVoltageRegulator/*
git add moskitoPlate/*
git add nixi/nixitest_circuit_schematic/
git add nixi/nixitest-AtTiny-code/
git add stm32-libs/*
git checkout -f .metadata/
git status
