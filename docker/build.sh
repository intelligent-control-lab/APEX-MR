#!/bin/bash

# Script to build Docker image with Gurobi and GitHub credentials

# Exit on any error
set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to check if a file exists
check_file() {
    if [ ! -f "$1" ]; then
        echo -e "${RED}Error: File $1 not found${NC}"
        exit 1
    fi
}

# Function to read Gurobi license file
read_gurobi_license() {
    local license_file="$1"
    
    # Read the license file line by line
    while IFS= read -r line; do
        if [[ $line == WLSACCESSID=* ]]; then
            GUROBI_WLSACCESSID="${line#WLSACCESSID=}"
        elif [[ $line == WLSSECRET=* ]]; then
            GUROBI_WLSSECRET="${line#WLSSECRET=}"
        elif [[ $line == LICENSEID=* ]]; then
            GUROBI_LICENSEID="${line#LICENSEID=}"
        fi
    done < "$license_file"
    
    # Verify all required values were found
    if [ -z "$GUROBI_WLSACCESSID" ] || [ -z "$GUROBI_WLSSECRET" ] || [ -z "$GUROBI_LICENSEID" ]; then
        echo -e "${RED}Error: Missing required Gurobi license information${NC}"
        exit 1
    fi
}

# Check for GitHub token
if [ -z "$GITHUB_TOKEN" ]; then
    echo -e "${RED}Error: GITHUB_TOKEN environment variable not set${NC}"
    exit 1
fi

# Look for Gurobi license in common locations
GUROBI_LICENSE_LOCATIONS=(
    "/opt/gurobi1102/gurobi.lic"
    "$HOME/.gurobi/gurobi.lic"
    "./gurobi.lic"
)

FOUND_LICENSE=false
for license_location in "${GUROBI_LICENSE_LOCATIONS[@]}"; do
    if [ -f "$license_location" ]; then
        echo -e "${GREEN}Found Gurobi license at: $license_location${NC}"
        read_gurobi_license "$license_location"
        FOUND_LICENSE=true
        break
    fi
done

if [ "$FOUND_LICENSE" = false ]; then
    echo -e "${RED}Error: Could not find Gurobi license file in common locations${NC}"
    exit 1
fi

# Build the Docker image
echo -e "${GREEN}Building Docker image with credentials...${NC}"
docker build \
    --build-arg GITHUB_TOKEN="$GITHUB_TOKEN" \
    --build-arg GUROBI_WLSACCESSID="$GUROBI_WLSACCESSID" \
    --build-arg GUROBI_WLSSECRET="$GUROBI_WLSSECRET" \
    --build-arg GUROBI_LICENSEID="$GUROBI_LICENSEID" \
    -t apex_mr-image .

echo -e "${GREEN}Docker image built successfully!${NC}"