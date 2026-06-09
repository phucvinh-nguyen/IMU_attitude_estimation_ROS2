#!/bin/bash
#
# Build and test script for IMU pre-filtering package
# Usage: ./build.sh [command]
#   build  - Build the package (default)
#   test   - Run unit tests
#   clean  - Clean build artifacts
#   help   - Show this help message

set -e

WORKSPACE_DIR="/home/victor/proposed_ws"
PACKAGE_NAME="pre_filtering_imu"

cd "$WORKSPACE_DIR"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

build_package() {
    print_status "Building $PACKAGE_NAME package..."
    
    if [ ! -f setup.bash ]; then
        print_warning "setup.bash not found. Running catkin_init_workspace..."
        catkin_init_workspace src/
    fi
    
    source setup.bash 2>/dev/null || true
    
    print_status "Running catkin build..."
    if catkin build "$PACKAGE_NAME" --no-status; then
        print_status "Build successful!"
        source devel/setup.bash
        print_status "Workspace sourced. Ready to use!"
        return 0
    else
        print_error "Build failed!"
        return 1
    fi
}

run_tests() {
    print_status "Running unit tests..."
    
    if [ ! -f devel/setup.bash ]; then
        print_error "Workspace not built. Run './build.sh build' first."
        return 1
    fi
    
    source devel/setup.bash
    
    # Run Python tests
    if [ -f src/$PACKAGE_NAME/test/test_filters.py ]; then
        print_status "Running Python tests..."
        python src/$PACKAGE_NAME/test/test_filters.py
    fi
    
    print_status "Tests completed!"
}

clean_build() {
    print_status "Cleaning build artifacts..."
    rm -rf build/ devel/ .catkin_tools/
    print_status "Clean complete!"
}

show_help() {
    cat << EOF
Usage: ./build.sh [command]

Commands:
  build   Build the $PACKAGE_NAME package (default)
  test    Run unit tests
  clean   Remove build artifacts
  help    Show this message

Examples:
  ./build.sh              # Build the package
  ./build.sh build        # Same as above
  ./build.sh test         # Run tests
  ./build.sh clean        # Clean build files

After successful build, source the workspace:
  source devel/setup.bash

Then launch the node:
  roslaunch $PACKAGE_NAME imu_prefilter.launch

EOF
}

# Main
case "${1:-build}" in
    build)
        build_package
        ;;
    test)
        run_tests
        ;;
    clean)
        clean_build
        ;;
    help)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
