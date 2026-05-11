set -e  

# Configuration
SCRIPT_DIR="$HOME/ros2_sensor_tactile/universal_manipulation_interface-main"
DATA_DIR="$HOME/ros2_sensor_tactile/umi_data"
FRAME_SKIP=1 # Check every frame for thorough detection

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "===================================="
echo " BATCH QR DETECTION FOR ALL DEMOS"
echo "=========================================="
echo ""
echo " Data directory: $DATA_DIR"
echo " Frame skip: $FRAME_SKIP"
echo ""

# Check if data directory exists
if [ ! -d "$DATA_DIR" ]; then
    echo -e "${RED} Error: Data directory not found: $DATA_DIR${NC}"
    exit 1
fi

# Check if detection script exists
DETECT_SCRIPT="$SCRIPT_DIR/detect_qr_from_video.py"
if [ ! -f "$DETECT_SCRIPT" ]; then
    echo -e "${RED} Error: Detection script not found: $DETECT_SCRIPT${NC}"
    exit 1
fi

# Find all demo directories
echo "🔍 Searching for demo directories..."
DEMO_DIRS=$(find "$DATA_DIR" -maxdepth 1 -type d -name "demo_*" | sort)

if [ -z "$DEMO_DIRS" ]; then
    echo -e "${RED} No demo directories found!${NC}"
    echo "Expected directories like: demo_001, demo_002, etc."
    exit 1
fi

DEMO_COUNT=$(echo "$DEMO_DIRS" | wc -l)
echo -e "${GREEN}✓ Found $DEMO_COUNT demo directories${NC}"
echo ""

# Process each demo
PROCESSED=0
SKIPPED=0
FAILED=0

for DEMO_DIR in $DEMO_DIRS; do
    DEMO_NAME=$(basename "$DEMO_DIR")
    
    echo "===================================="
    echo -e "${BLUE} Processing: $DEMO_NAME${NC}"
    echo "=========================================="
    
    # Find video file in this directory
    VIDEO_FILE=""
    
    # Try different video extensions
    for ext in MP4 mp4 MOV mov; do
        FOUND=$(find "$DEMO_DIR" -maxdepth 1 -type f -name "*.$ext" | head -n 1)
        if [ ! -z "$FOUND" ]; then
            VIDEO_FILE="$FOUND"
            break
        fi
    done
    
    if [ -z "$VIDEO_FILE" ]; then
        echo -e "${YELLOW}  No video file found in $DEMO_DIR${NC}"
        echo "   Skipping..."
        SKIPPED=$((SKIPPED + 1))
        echo ""
        continue
    fi
    
    echo " Video: $(basename "$VIDEO_FILE")"
    
    # Output path for QR sync data
    QR_OUTPUT="$DEMO_DIR/qr_sync_data.json"
    
    # Check if QR sync file already exists
    if [ -f "$QR_OUTPUT" ]; then
        echo -e "${YELLOW}  QR sync file already exists: $QR_OUTPUT${NC}"
        read -p "   Overwrite? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "   Skipping..."
            SKIPPED=$((SKIPPED + 1))
            echo ""
            continue
        fi
    fi
    
    # Run QR detection
    echo " Detecting QR codes..."
    
    if python "$DETECT_SCRIPT" \
        --video "$VIDEO_FILE" \
        --output "$QR_OUTPUT" \
        --frame_skip $FRAME_SKIP; then
        
        echo -e "${GREEN} Success: $QR_OUTPUT${NC}"
        PROCESSED=$((PROCESSED + 1))
        
        # Show brief summary
        if [ -f "$QR_OUTPUT" ]; then
            QR_COUNT=$(python -c "import json; data=json.load(open('$QR_OUTPUT')); print(len(data.get('all_qr_codes', [])))" 2>/dev/null || echo "?")
            echo "   QR codes detected: $QR_COUNT"
        fi
    else
        echo -e "${RED} Failed to process $DEMO_NAME${NC}"
        FAILED=$((FAILED + 1))
    fi
    
    echo ""
done

# Final summary
echo "====================================="
echo " PROCESSING COMPLETE"
echo "=========================================="
echo -e "${GREEN}Successfully processed: $PROCESSED${NC}"
if [ $SKIPPED -gt 0 ]; then
    echo -e "${YELLOW}  Skipped: $SKIPPED${NC}"
fi
if [ $FAILED -gt 0 ]; then
    echo -e "${RED} Failed: $FAILED${NC}"
fi
echo ""

# List all generated QR sync files
echo "Generated QR sync files:"
find "$DATA_DIR" -name "qr_sync_data.json" -type f | while read file; do
    demo=$(basename $(dirname "$file"))
    echo "   $demo: $file"
done
echo ""



echo "=========================================="