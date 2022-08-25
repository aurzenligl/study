#!/bin/bash

${CXX:-g++} -O2 -g -std=c++17 -Wall -pedantic -pthread -lpthread -lboost_system -lboost_thread child.cpp -ochild
${CXX:-g++} -O2 -g -std=c++17 -Wall -pedantic -pthread -lpthread -lboost_system -lboost_filesystem parent.cpp -oparent
