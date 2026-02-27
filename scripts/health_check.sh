#!/bin/bash
# Health check para monitoreo ATLAS NEXUS

set -e

echo "=== ATLAS NEXUS Health Check ==="
echo "Date: $(date)"
echo ""

# PUSH (8791)
echo "Checking PUSH (8791)..."
if command -v jq &>/dev/null; then
  curl -sf http://localhost:8791/health | jq -r '.score // .ok // "ok"' || echo "PUSH FAILED"
else
  curl -sf http://localhost:8791/health || echo "PUSH FAILED"
fi

# NEXUS (8000)
echo "Checking NEXUS (8000)..."
curl -sf http://localhost:8000/health 2>/dev/null || echo "NEXUS FAILED (or not running)"

# Robot (8002)
echo "Checking Robot (8002)..."
curl -sf http://localhost:8002/status 2>/dev/null || echo "ROBOT FAILED (or not running)"

# Autonomous health (si existe)
echo "Checking Autonomous Systems..."
curl -sf http://localhost:8791/health 2>/dev/null | jq -r '.score // "n/a"' 2>/dev/null || echo "AUTONOMOUS CHECK SKIP"

echo ""
echo "=== Health Check Complete ==="
