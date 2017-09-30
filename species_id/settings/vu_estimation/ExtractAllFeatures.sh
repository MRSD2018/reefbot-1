#!/bin/sh

parallel -j 1 bash -- 1s -1 settings/vu_estimation/*Extraction.sh
