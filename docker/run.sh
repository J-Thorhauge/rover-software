#!/usr/bin/env bash

# Enhanced run script for Docker Compose services
# Requires a specific service, a mode (--dev or --prod), and supports optional add-ons.

# --- Configuration ---
COMPOSE_FILE="docker-compose.yaml"

# --- Functions ---
print_usage() {
    echo "Usage: $0 [SERVICE] [MODE] [OPTIONS]"
    echo ""
    echo "This script runs specified services using Docker Compose."
    echo ""
    echo "REQUIRED:"
    echo "  SERVICE           The main service to run (e.g., rover, cameras)."
    echo "  MODE              The mode to run in:"
    echo "    --dev           Development mode (mounts source code)."
    echo "    --prod          Production mode (uses built image from 'deploy' stage)."
    echo ""
    echo "OPTIONS:"
    echo "  --triton          Include the triton-server service."
    echo "  --zenoh, -z       Include the zenoh-router service."
    echo "  --help, -h        Show this help message."
    echo ""
    echo "STANDALONE SERVICES (mode not required):"
    echo "  $0 zenoh              Start only the zenoh-router."
    echo "  $0 triton             Start only the triton-server."
    echo ""
    echo "EXAMPLES:"
    echo "  $0 rover --dev"
    echo "  $0 cameras --prod"
    echo "  $0 rover --dev --triton"
    echo "  $0 rover --prod --zenoh --triton"
}

# --- Argument Parsing ---

# Check for no arguments or help flag
if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
    print_usage
    exit 0
fi

# --- Main Logic ---

# Initialize variables
SERVICE_NAME="$1"
shift # Consume the service name argument
MODE=""
TRITON=false
ZENOH=false
SERVICES=""

# Handle standalone services that don't require a mode
if [[ "$SERVICE_NAME" == "zenoh" && $# -eq 0 ]]; then
    echo "[INFO] Starting zenoh-router only..."
    docker compose -f "$COMPOSE_FILE" up zenoh-router --detach --build --remove-orphans
    echo "[INFO] Zenoh router started. Use 'docker compose -f $COMPOSE_FILE logs -f zenoh-router' for logs."
    exit 0
fi

if [[ "$SERVICE_NAME" == "triton" && $# -eq 0 ]]; then
    echo "[INFO] Starting triton-server only..."
    docker compose -f "$COMPOSE_FILE" up triton-server --detach --build --remove-orphans
    echo "[INFO] Triton server started. Use 'docker compose -f $COMPOSE_FILE logs -f triton-server' for logs."
    exit 0
fi

# For main services, a mode is required
if [[ $# -eq 0 ]]; then
    echo "[ERROR] Mode (--dev or --prod) is required for service '$SERVICE_NAME'." >&2
    print_usage
    exit 1
fi

# Parse mode
case $1 in
    --dev)
        MODE="dev"
        SERVICES="$SERVICE_NAME" # In dev, service name matches compose definition
        shift
        ;;
    --prod)
        MODE="prod"
        SERVICES="$SERVICE_NAME-deploy perception_container" # In prod, service has '-deploy' suffix
        shift
        ;;
    *)
        echo "[ERROR] Unknown or missing mode: '$1'. Expected --dev or --prod." >&2
        print_usage
        exit 1
        ;;
esac

# Parse remaining optional arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --triton)
            TRITON=true
            shift
            ;;
        --zenoh|-z)
            ZENOH=true
            shift
            ;;
        *)
            echo "[ERROR] Unknown option: $1" >&2
            print_usage
            exit 1
            ;;
    esac
done

# --- Service Assembly & Execution ---

echo "[INFO] Running service '$SERVICE_NAME' in '$MODE' mode..."

# Add optional services to the list
if [ "$TRITON" = true ]; then
    SERVICES="$SERVICES triton-server"
    echo "[INFO] Including triton-server..."
fi

if [ "$ZENOH" = true ]; then
    SERVICES="$SERVICES zenoh-router"
    echo "[INFO] Including zenoh-router..."
fi

echo "[INFO] Starting services: $SERVICES"

# Ensure iptable_raw module is loaded (often needed for Docker networking)
sudo modprobe iptable_raw 2>/dev/null || true

# Execute docker compose
docker compose -f "$COMPOSE_FILE" up $SERVICES --detach --build --remove-orphans

# --- Post-run Information ---
echo ""
echo "[INFO] ✅ Launch complete!"
echo "[INFO] Use 'docker compose -f $COMPOSE_FILE logs -f' to view logs of all running services."
echo "[INFO] Use 'docker compose -f $COMPOSE_FILE down' to stop all services."
echo "[INFO] To access the main container, run: docker exec -it ${SERVICE_NAME} bash"