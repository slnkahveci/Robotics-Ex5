#!/bin/bash

# Interactive script to compare RrtConConBase and YourPlanner
# This script helps you switch between planners and compare results

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$SCRIPT_DIR/build"
HEADER_FILE="$SCRIPT_DIR/TutorialPlanSystem.h"
BENCHMARK_FILE="$BUILD_DIR/benchmark.csv"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

clear
echo -e "${BLUE}╔════════════════════════════════════════════╗"
echo -e "║   Planner Comparison Helper Script        ║"
echo -e "╚════════════════════════════════════════════╝${NC}"
echo ""

# Backup the original header file
echo -e "${YELLOW}Creating backup of TutorialPlanSystem.h...${NC}"
cp "$HEADER_FILE" "$HEADER_FILE.backup"

# Function to restore original file
cleanup() {
    if [ -f "$HEADER_FILE.backup" ]; then
        echo -e "\n${YELLOW}Restoring original TutorialPlanSystem.h...${NC}"
        mv "$HEADER_FILE.backup" "$HEADER_FILE"
        echo -e "${GREEN}Original file restored.${NC}"
    fi
}
trap cleanup EXIT

# Function to switch planner
switch_to_planner() {
    local planner_type=$1
    
    if [ "$planner_type" = "baseline" ]; then
        echo -e "${CYAN}Switching to RrtConConBase (baseline)...${NC}"
        sed -i.tmp 's|#include "YourPlanner.h"|#include "RrtConConBase.h"|' "$HEADER_FILE"
        sed -i.tmp 's|YourPlanner planner;|RrtConConBase planner;  //Baseline planner|' "$HEADER_FILE"
    else
        echo -e "${CYAN}Switching to YourPlanner...${NC}"
        sed -i.tmp 's|#include "RrtConConBase.h"|#include "YourPlanner.h"|' "$HEADER_FILE"
        sed -i.tmp 's|RrtConConBase planner;.*|YourPlanner planner;  //The implementation of your planner|' "$HEADER_FILE"
    fi
    rm -f "$HEADER_FILE.tmp"
    
    echo -e "${YELLOW}Compiling...${NC}"
    cd "$BUILD_DIR"
    if make -j4 2>&1 | grep -q "error:"; then
        echo -e "${RED}Compilation failed! Check for errors.${NC}"
        make
        return 1
    else
        echo -e "${GREEN}Compilation successful!${NC}"
        return 0
    fi
}

# Function to display current planner
show_current_planner() {
    if grep -q "YourPlanner planner" "$HEADER_FILE"; then
        echo -e "${GREEN}Current planner: YourPlanner${NC}"
    elif grep -q "RrtConConBase planner" "$HEADER_FILE"; then
        echo -e "${GREEN}Current planner: RrtConConBase (baseline)${NC}"
    else
        echo -e "${YELLOW}Current planner: Unknown${NC}"
    fi
}

# Function to show recent results
show_results() {
    if [ ! -f "$BENCHMARK_FILE" ]; then
        echo -e "${RED}No benchmark.csv file found yet.${NC}"
        return
    fi
    
    echo -e "\n${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${BLUE}Recent Benchmark Results (last 5)${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}"
    echo ""
    echo "Date       | Time        | Solved | Planner        | Vertices | Queries | Runtime(ms)"
    echo "-----------|-------------|--------|----------------|----------|---------|------------"
    
    tail -5 "$BENCHMARK_FILE" | while IFS=',' read -r date time solved planner vertices col_q free_q runtime; do
        printf "%-10s | %-11s | %-6s | %-14s | %-8s | %-7s | %s\n" \
            "$date" "$time" "$solved" "$planner" "$vertices" "$col_q" "$runtime"
    done
    echo ""
}

# Function to compare last two results
compare_last_two() {
    if [ ! -f "$BENCHMARK_FILE" ]; then
        echo -e "${RED}No benchmark.csv file found yet.${NC}"
        return
    fi
    
    local lines=$(wc -l < "$BENCHMARK_FILE")
    if [ $lines -lt 2 ]; then
        echo -e "${YELLOW}Need at least 2 benchmark entries to compare.${NC}"
        return
    fi
    
    echo -e "\n${BLUE}═══════════════════════════════════════════${NC}"
    echo -e "${BLUE}Comparison of Last Two Results${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════${NC}\n"
    
    local result1=$(tail -2 "$BENCHMARK_FILE" | head -1)
    local result2=$(tail -1 "$BENCHMARK_FILE")
    
    IFS=',' read -r date1 time1 solved1 planner1 vertices1 col_q1 free_q1 runtime1 <<< "$result1"
    IFS=',' read -r date2 time2 solved2 planner2 vertices2 col_q2 free_q2 runtime2 <<< "$result2"
    
    echo -e "${CYAN}First Result:${NC}"
    echo "  Planner: $planner1"
    echo "  Solved: $solved1"
    echo "  Vertices: $vertices1"
    echo "  Collision Queries: $col_q1"
    echo "  Runtime: $runtime1 ms"
    echo ""
    
    echo -e "${CYAN}Second Result:${NC}"
    echo "  Planner: $planner2"
    echo "  Solved: $solved2"
    echo "  Vertices: $vertices2"
    echo "  Collision Queries: $col_q2"
    echo "  Runtime: $runtime2 ms"
    echo ""
    
    if [ "$solved1" = "true" ] && [ "$solved2" = "true" ]; then
        echo -e "${GREEN}Both planners found a solution!${NC}\n"
        
        # Compare vertices
        if [ "$vertices1" -lt "$vertices2" ]; then
            echo -e "${GREEN}✓${NC} $planner1 used fewer vertices ($vertices1 vs $vertices2)"
        elif [ "$vertices1" -gt "$vertices2" ]; then
            echo -e "${GREEN}✓${NC} $planner2 used fewer vertices ($vertices2 vs $vertices1)"
        else
            echo "  Both used the same number of vertices"
        fi
        
        # Compare runtime
        if (( $(echo "$runtime1 < $runtime2" | bc -l 2>/dev/null || echo 0) )); then
            local diff=$(echo "$runtime2 - $runtime1" | bc 2>/dev/null || echo "N/A")
            echo -e "${GREEN}✓${NC} $planner1 was faster by $diff ms"
        elif (( $(echo "$runtime1 > $runtime2" | bc -l 2>/dev/null || echo 0) )); then
            local diff=$(echo "$runtime1 - $runtime2" | bc 2>/dev/null || echo "N/A")
            echo -e "${GREEN}✓${NC} $planner2 was faster by $diff ms"
        else
            echo "  Both had the same runtime"
        fi
        
        # Compare collision queries
        if [ "$col_q1" -lt "$col_q2" ]; then
            echo -e "${GREEN}✓${NC} $planner1 used fewer collision queries ($col_q1 vs $col_q2)"
        elif [ "$col_q1" -gt "$col_q2" ]; then
            echo -e "${GREEN}✓${NC} $planner2 used fewer collision queries ($col_q2 vs $col_q1)"
        else
            echo "  Both used the same number of collision queries"
        fi
    fi
    echo ""
}

# Main menu
while true; do
    echo ""
    show_current_planner
    echo ""
    echo -e "${YELLOW}What would you like to do?${NC}"
    echo "  1) Switch to RrtConConBase (baseline) and compile"
    echo "  2) Switch to YourPlanner and compile"
    echo "  3) Run current planner (opens GUI)"
    echo "  4) Show recent results"
    echo "  5) Compare last two results"
    echo "  6) Exit"
    echo ""
    read -p "Enter choice [1-6]: " choice
    
    case $choice in
        1)
            switch_to_planner "baseline"
            echo -e "${GREEN}Ready to test! Choose option 3 to run.${NC}"
            ;;
        2)
            switch_to_planner "your"
            echo -e "${GREEN}Ready to test! Choose option 3 to run.${NC}"
            ;;
        3)
            echo -e "${CYAN}Starting planner...${NC}"
            echo -e "${YELLOW}Please use the GUI to click 'Start Planning' when ready.${NC}"
            echo -e "${YELLOW}Results will be saved automatically to benchmark.csv${NC}"
            cd "$BUILD_DIR"
            ./tutorialPlan
            echo -e "${GREEN}Program closed.${NC}"
            ;;
        4)
            show_results
            ;;
        5)
            compare_last_two
            ;;
        6)
            echo -e "${GREEN}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice. Please try again.${NC}"
            ;;
    esac
done
