#!/usr/bin/env python3
"""
ROS2 AMR API Server - FIXED VERSION
A FastAPI server with a simple, controllable robot simulation.
"""

import asyncio
import json
import logging
import math
import time
from datetime import datetime
from typing import Dict, Any, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="ROS2 AMR API Server", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3001", "http://localhost:3000", "http://localhost:3002"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global state
start_time = time.time()  # System start time for uptime calculation

robot_status = {
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "velocity": {"linear_x": 0.0, "angular_z": 0.0},
    "battery_level": 100.0,
    "mission_state": "idle",
    "goal_state": "none",
    "timestamp": time.time()
}

metrics = {
    "goals_completed": 0,
    "total_replans": 0,
    "distance_traveled": 0.0,
    "success_rate": 0.0,
    "system_uptime": 0.0,
    "avg_speed": 0.0,
    "goals_per_hour": 0.0,
    "replan_rate": 0.0,
    "total_missions": 0
}

# User-editable map elements
user_goals = []
user_obstacles = []
robot_trail = []

current_mission_id = None
active_connections: List[WebSocket] = []

# Demo missions
demo_missions = {
    "demo_mission_001": {
        "id": "demo_mission_001",
        "name": "Demo Navigation Mission",
        "description": "Navigate through waypoints around obstacles",
        "goals": [],  # Will be generated dynamically
        "obstacles": [],  # Will be generated dynamically
        "status": "pending",
        "created_at": datetime.now().isoformat()
    }
}

def generate_demo_mission():
    """Generate a demo mission with strategic obstacles placed in robot's path for navigation showcase."""
    import random
    
    # Create strategic obstacles DIRECTLY IN THE ROBOT'S PATH - forces navigation around them
    obstacles = [
        {"x": 2.5, "y": 1.5, "type": "rectangle", "size": 0.8},   # DIRECTLY blocks path from (0,0) to Goal 1 (5.0, 3.0)
        {"x": 6.5, "y": 2.0, "type": "rectangle", "size": 0.8},   # DIRECTLY blocks path from Goal 1 to Goal 2 (8.0, 1.0)
        {"x": 3.0, "y": -1.0, "type": "rectangle", "size": 0.8}   # DIRECTLY blocks path from Goal 2 to Goal 3 (4.0, -4.0)
    ]
    
    # Generate goals that require going around obstacles - showcase navigation
    goals = [
        {"x": 5.0, "y": 3.0, "theta": 0.0},    # Goal 1: robot must go around obstacle at (3.0, 1.5)
        {"x": 8.0, "y": 1.0, "theta": 1.57},   # Goal 2: robot must go around obstacle at (6.0, 0.5)
        {"x": 4.0, "y": -4.0, "theta": 3.14},  # Goal 3: robot must go around obstacle at (2.0, -2.0)
        {"x": -2.0, "y": -2.0, "theta": -1.57}, # Goal 4: clear path
        {"x": -4.0, "y": 2.0, "theta": 0.0},   # Goal 5: clear path
        {"x": 0.0, "y": 4.0, "theta": -1.57},  # Goal 6: clear path
        {"x": 0.0, "y": 0.0, "theta": 0.0}     # Return home
    ]
    
    return obstacles, goals

# Pydantic models
class MissionGoal(BaseModel):
    x: float
    y: float
    theta: float

class Mission(BaseModel):
    name: str
    goals: List[MissionGoal]

# API Endpoints
@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "ROS2 AMR API Server",
        "version": "1.0.0",
        "endpoints": {
            "mission": "/api/mission",
            "status": "/api/status",
            "metrics": "/api/metrics",
            "websocket": "/ws"
        }
    }

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}

@app.get("/api/status")
async def get_robot_status():
    """Get current robot status."""
    return robot_status

@app.get("/api/missions")
async def get_missions():
    """Get all available missions."""
    # Always regenerate demo mission to ensure it has obstacles and new coordinates
    obstacles, goals = generate_demo_mission()
    logger.info(f"Generated obstacles: {obstacles}")
    logger.info(f"Generated goals: {goals}")
    
    # Update the demo mission with new data
    demo_missions["demo_mission_001"]["goals"] = goals
    demo_missions["demo_mission_001"]["obstacles"] = obstacles
    demo_missions["demo_mission_001"]["status"] = "pending"
    
    # If no mission is currently running, reset the current_mission_id to allow new mission
    global current_mission_id
    if current_mission_id is None or current_mission_id not in demo_missions:
        current_mission_id = None
    
    return {"missions": list(demo_missions.values())}

@app.get("/api/map/elements")
async def get_map_elements():
    """Get all map elements (goals, obstacles, trail)."""
    return {
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    }

@app.post("/api/map/goals")
async def add_goal(goal: MissionGoal):
    """Add a user goal to the map."""
    global user_goals
    user_goals.append({"x": goal.x, "y": goal.y, "theta": goal.theta})
    
    await broadcast_to_clients({
        "type": "map_update",
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    })
    
    return {"status": "goal_added", "goals": user_goals}

@app.post("/api/map/obstacles")
async def add_obstacle(obstacle: dict):
    """Add a user obstacle to the map."""
    global user_obstacles
    user_obstacles.append({
        "x": obstacle["x"],
        "y": obstacle["y"],
        "type": obstacle.get("type", "rectangle"),
        "size": obstacle.get("size", 0.5)
    })
    
    await broadcast_to_clients({
        "type": "map_update",
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    })
    
    return {"status": "obstacle_added", "obstacles": user_obstacles}

@app.delete("/api/map/goals")
async def clear_goals():
    """Clear all user goals."""
    global user_goals
    user_goals = []
    
    await broadcast_to_clients({
        "type": "map_update",
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    })
    
    return {"status": "goals_cleared"}

@app.delete("/api/map/obstacles")
async def clear_obstacles():
    """Clear all user obstacles."""
    global user_obstacles
    user_obstacles = []
    
    await broadcast_to_clients({
        "type": "map_update",
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    })
    
    return {"status": "obstacles_cleared"}

@app.delete("/api/map/trail")
async def clear_trail():
    """Clear the robot trail."""
    global robot_trail
    robot_trail = []
    
    await broadcast_to_clients({
        "type": "map_update",
        "goals": user_goals,
        "obstacles": user_obstacles,
        "trail": robot_trail
    })
    
    return {"status": "trail_cleared"}

@app.options("/api/mission")
async def options_mission():
    """Handle CORS preflight for mission creation."""
    return {"message": "OK"}

@app.post("/api/mission")
async def create_mission(mission: Mission):
    """Create a new mission."""
    mission_id = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    demo_missions[mission_id] = {
        "id": mission_id,
        "name": mission.name,
        "description": f"Custom mission with {len(mission.goals)} goals",
        "goals": [{"x": goal.x, "y": goal.y, "theta": goal.theta} for goal in mission.goals],
        "status": "pending",
        "created_at": datetime.now().isoformat()
    }
    
    logger.info(f"Created mission {mission_id} with {len(mission.goals)} goals")
    return {"mission_id": mission_id, "status": "created"}

@app.post("/api/mission/{mission_id}/start")
async def start_mission(mission_id: str):
    """Start a specific mission."""
    global current_mission_id
    
    if mission_id not in demo_missions:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    current_mission_id = mission_id
    demo_missions[mission_id]["status"] = "running"
    
    # Broadcast mission data to frontend (including obstacles)
    mission = demo_missions[mission_id]
    await broadcast_to_clients({
        "type": "mission_started",
        "mission_id": mission_id,
        "mission": {
            "goals": mission["goals"],
            "obstacles": mission["obstacles"]
        },
        "robot_status": robot_status,
        "metrics": metrics
    })
    
    # Start mission simulation
    asyncio.create_task(simulate_mission(mission_id))
    
    logger.info(f"Starting mission {mission_id}")
    return {"status": "started", "mission_id": mission_id}

@app.post("/api/mission/start")
async def start_generic_mission():
    """Start a mission using user goals."""
    global current_mission_id, user_goals
    
    if not user_goals:
        raise HTTPException(status_code=400, detail="No goals available. Please add goals to the map.")
    
    # Create a temporary mission with user goals
    mission_id = f"user_mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    demo_missions[mission_id] = {
        "id": mission_id,
        "name": "User Mission",
        "description": f"Mission with {len(user_goals)} user-defined goals",
        "goals": user_goals.copy(),
        "status": "running",
        "created_at": datetime.now().isoformat()
    }
    
    current_mission_id = mission_id
    
    # Start mission simulation
    asyncio.create_task(simulate_mission(mission_id))
    
    logger.info(f"Starting user mission {mission_id} with {len(user_goals)} goals")
    return {"status": "started", "mission_id": mission_id, "goals": len(user_goals)}

@app.post("/api/mission/stop")
async def stop_mission():
    """Stop current mission."""
    global current_mission_id
    
    if current_mission_id:
        demo_missions[current_mission_id]["status"] = "completed"
        logger.info(f"Stopping mission {current_mission_id}")
        current_mission_id = None
    
    return {"status": "stopped"}

@app.post("/api/robot/reset")
async def reset_robot():
    """Reset robot to origin and clear metrics."""
    global robot_status, metrics, current_mission_id, user_goals, robot_trail
    
    # Stop any active mission
    if current_mission_id:
        logger.info(f"Stopping mission {current_mission_id} for reset")
        demo_missions[current_mission_id]["status"] = "completed"
        current_mission_id = None
    
    # Reset robot to origin immediately
    robot_status.update({
        "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "velocity": {"linear_x": 0.0, "angular_z": 0.0},
        "battery_level": 100.0,
        "mission_state": "idle",
        "goal_state": "none",
        "timestamp": time.time()
    })
    
    # Update system uptime (don't reset cumulative metrics)
    current_time = time.time()
    metrics["system_uptime"] = current_time - start_time
    
    # Update performance summary calculations
    if metrics["system_uptime"] > 0:
        metrics["goals_per_hour"] = (metrics["goals_completed"] / (metrics["system_uptime"] / 3600))
        metrics["replan_rate"] = (metrics["total_replans"] / (metrics["system_uptime"] / 60))
    
    # Calculate average speed based on distance and time
    if metrics["system_uptime"] > 0:
        metrics["avg_speed"] = metrics["distance_traveled"] / (metrics["system_uptime"] / 3600)  # m/h
    
    # Clear user goals and trail
    user_goals = []
    robot_trail = []
    
    logger.info("Robot reset to origin, metrics cleared, and trail/goals cleared")
    
    # Broadcast reset to WebSocket clients
    await broadcast_to_clients({
        "type": "robot_reset",
        "robot_status": robot_status,
        "metrics": metrics,
        "goals": user_goals,
        "trail": robot_trail,
        "timestamp": datetime.now().isoformat()
    })
    
    return {"status": "reset", "message": "Robot reset to origin (0,0), metrics cleared, and trail/goals cleared"}

@app.get("/api/metrics")
async def get_metrics():
    """Get current performance metrics."""
    return metrics

# WebSocket endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time communication."""
    await websocket.accept()
    active_connections.append(websocket)
    logger.info(f"WebSocket client connected. Total connections: {len(active_connections)}")
    
    try:
        while True:
            # Keep connection alive and send periodic updates
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        active_connections.remove(websocket)
        logger.info(f"WebSocket client disconnected. Total connections: {len(active_connections)}")

async def broadcast_to_clients(message: Dict[str, Any]):
    """Broadcast message to all connected WebSocket clients."""
    if not active_connections:
        return
    
    message_text = json.dumps(message)
    disconnected = []
    
    for connection in active_connections:
        try:
            await connection.send_text(message_text)
        except:
            disconnected.append(connection)
    
    # Remove disconnected clients
    for connection in disconnected:
        active_connections.remove(connection)

# Simple mission simulation
async def simulate_mission(mission_id: str):
    """Simulate mission execution."""
    global robot_status, metrics, current_mission_id, robot_trail
    
    if mission_id not in demo_missions:
        return
    
    mission = demo_missions[mission_id]
    goals = mission["goals"]
    
    for i, goal in enumerate(goals):
        # Check if mission is still active
        if current_mission_id != mission_id:
            break
        
        # Move to goal
        await move_to_goal(goal["x"], goal["y"], goal["theta"])
        
        # Update metrics with real data
        metrics["goals_completed"] += 1
        # Calculate actual distance traveled
        if i > 0:
            prev_goal = goals[i-1]
            distance = ((goal["x"] - prev_goal["x"])**2 + (goal["y"] - prev_goal["y"])**2)**0.5
            metrics["distance_traveled"] += distance
        else:
            # Distance from origin to first goal
            distance = (goal["x"]**2 + goal["y"]**2)**0.5
            metrics["distance_traveled"] += distance
        
        # Update system uptime
        current_time = time.time()
        metrics["system_uptime"] = current_time - start_time
        
        # Update performance summary calculations
        if metrics["system_uptime"] > 0:
            metrics["goals_per_hour"] = (metrics["goals_completed"] / (metrics["system_uptime"] / 3600))
            metrics["replan_rate"] = (metrics["total_replans"] / (metrics["system_uptime"] / 60))
            metrics["avg_speed"] = metrics["distance_traveled"] / (metrics["system_uptime"] / 3600)  # m/h
            
            # Calculate success rate (cap at 100%)
            total_possible_goals = metrics["total_missions"] * 7  # 7 goals per demo mission
            if total_possible_goals > 0:
                success_rate = (metrics["goals_completed"] / total_possible_goals) * 100
                metrics["success_rate"] = min(success_rate, 100.0)  # Cap at 100%
        
        # Broadcast update
        await broadcast_to_clients({
            "type": "goal_reached",
            "goal_index": i,
            "robot_status": robot_status,
            "metrics": metrics,
            "trail": robot_trail
        })
        
        await asyncio.sleep(1)  # Pause between goals
    
    # Mission completed
    if current_mission_id == mission_id:
        current_mission_id = None
        demo_missions[mission_id]["status"] = "completed"
        
        # Update mission completion metrics
        metrics["total_missions"] += 1
        
        # Calculate cumulative success rate (goals completed vs total possible goals)
        # For demo mission: 7 goals per mission
        total_possible_goals = metrics["total_missions"] * 7  # 7 goals per demo mission
        if total_possible_goals > 0:
            success_rate = (metrics["goals_completed"] / total_possible_goals) * 100
            metrics["success_rate"] = min(success_rate, 100.0)  # Cap at 100%
        else:
            metrics["success_rate"] = 0.0
        
        # Update final performance metrics
        current_time = time.time()
        metrics["system_uptime"] = current_time - start_time
        if metrics["system_uptime"] > 0:
            metrics["goals_per_hour"] = (metrics["goals_completed"] / (metrics["system_uptime"] / 3600))
            metrics["replan_rate"] = (metrics["total_replans"] / (metrics["system_uptime"] / 60))
            metrics["avg_speed"] = metrics["distance_traveled"] / (metrics["system_uptime"] / 3600)  # m/h
        
        # Auto-reset after demo mission (only reset robot position, not metrics)
        if mission_id == "demo_mission_001":
            await asyncio.sleep(2)  # Brief pause
            # Reset robot position but keep cumulative metrics
            robot_status.update({
                "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
                "velocity": {"linear_x": 0.0, "angular_z": 0.0},
                "mission_state": "idle",
                "goal_state": "none",
                "timestamp": time.time()
            })
            robot_trail = []  # Clear trail
        
        await broadcast_to_clients({
            "type": "mission_completed",
            "mission_id": mission_id,
            "robot_status": robot_status,
            "metrics": metrics,
            "trail": robot_trail
        })

def is_collision_with_obstacles(x: float, y: float, obstacles: list) -> bool:
    """Check if a position collides with any obstacle."""
    for obstacle in obstacles:
        if obstacle["type"] == "circle":
            dist = ((x - obstacle["x"])**2 + (y - obstacle["y"])**2)**0.5
            if dist < obstacle["size"] + 0.5:  # Increased robot radius buffer
                return True
        else:  # rectangle
            dx = abs(x - obstacle["x"])
            dy = abs(y - obstacle["y"])
            if dx < obstacle["size"] + 0.5 and dy < obstacle["size"] + 0.5:
                return True
    return False

def generate_path_around_obstacles(start_x: float, start_y: float, target_x: float, target_y: float, obstacles: list) -> list:
    """Generate a path that avoids obstacles using simple path planning."""
    import math
    
    # Simple path planning: try direct path first, then use waypoints
    path_points = []
    
    # Check if direct path is clear
    direct_clear = True
    steps = 20
    for i in range(steps + 1):
        progress = i / steps
        x = start_x + (target_x - start_x) * progress
        y = start_y + (target_y - start_y) * progress
        if is_collision_with_obstacles(x, y, obstacles):
            direct_clear = False
            break
    
    if direct_clear:
        # Direct path is clear
        for i in range(steps + 1):
            progress = i / steps
            x = start_x + (target_x - start_x) * progress
            y = start_y + (target_y - start_y) * progress
            path_points.append({"x": x, "y": y})
    else:
        # Need to navigate around obstacles
        # Simple strategy: go to intermediate waypoints
        mid_x = (start_x + target_x) / 2
        mid_y = (start_y + target_y) / 2
        
        # Try to find a clear intermediate point
        for offset in [1.0, 1.5, 2.0, -1.0, -1.5, -2.0]:
            # Try perpendicular offset
            dx = target_x - start_x
            dy = target_y - start_y
            length = (dx**2 + dy**2)**0.5
            if length > 0:
                perp_x = -dy / length * offset
                perp_y = dx / length * offset
                
                waypoint_x = mid_x + perp_x
                waypoint_y = mid_y + perp_y
                
                if not is_collision_with_obstacles(waypoint_x, waypoint_y, obstacles):
                    # Found a clear waypoint, create path through it
                    # First segment: start to waypoint
                    for i in range(steps // 2 + 1):
                        progress = i / (steps // 2)
                        x = start_x + (waypoint_x - start_x) * progress
                        y = start_y + (waypoint_y - start_y) * progress
                        path_points.append({"x": x, "y": y})
                    
                    # Second segment: waypoint to target
                    for i in range(steps // 2 + 1):
                        progress = i / (steps // 2)
                        x = waypoint_x + (target_x - waypoint_x) * progress
                        y = waypoint_y + (target_y - waypoint_y) * progress
                        if i > 0:  # Skip first point to avoid duplicate
                            path_points.append({"x": x, "y": y})
                    break
    
    return path_points

async def move_to_goal(target_x: float, target_y: float, target_theta: float):
    """Move robot to a specific goal with obstacle avoidance."""
    global robot_status, robot_trail
    
    # Get current position
    current_x = robot_status["pose"]["x"]
    current_y = robot_status["pose"]["y"]
    current_theta = robot_status["pose"]["theta"]
    
    # Get obstacles from current mission
    obstacles = []
    if current_mission_id and current_mission_id in demo_missions:
        obstacles = demo_missions[current_mission_id].get("obstacles", [])
    
    # Generate path around obstacles
    path_points = generate_path_around_obstacles(current_x, current_y, target_x, target_y, obstacles)
    
    if not path_points:
        # Fallback to direct path if path planning fails
        dx = target_x - current_x
        dy = target_y - current_y
        steps = 20
        for step in range(steps + 1):
            progress = step / steps
            x = current_x + dx * progress
            y = current_y + dy * progress
            path_points.append({"x": x, "y": y})
    
    # Move along the planned path
    for i, point in enumerate(path_points):
        # Check if mission is still active
        if current_mission_id is None:
            break
        
        # Calculate orientation towards next point
        if i < len(path_points) - 1:
            next_point = path_points[i + 1]
            theta = math.atan2(next_point["y"] - point["y"], next_point["x"] - point["x"])
        else:
            theta = target_theta
        
        # Add position to trail
        robot_trail.append({"x": point["x"], "y": point["y"], "timestamp": time.time()})
        
        # Keep trail length manageable (last 100 points)
        if len(robot_trail) > 100:
            robot_trail = robot_trail[-100:]
        
        # Calculate velocity based on distance to next point
        if i < len(path_points) - 1:
            next_point = path_points[i + 1]
            distance = ((next_point["x"] - point["x"])**2 + (next_point["y"] - point["y"])**2)**0.5
            linear_x = min(distance * 15, 1.0)  # Increased speed, cap at 1.0 m/s
        else:
            linear_x = 0.3  # Faster final approach
        
        angular_z = 0.6  # Faster turning speed
        
        # Update robot status
        robot_status.update({
            "pose": {"x": point["x"], "y": point["y"], "theta": theta},
            "velocity": {"linear_x": linear_x, "angular_z": angular_z},
            "mission_state": "running",
            "goal_state": "navigating",
            "timestamp": time.time()
        })
        
        # Broadcast update
        await broadcast_to_clients({
            "type": "status_update",
            "robot_status": robot_status,
            "metrics": metrics,
            "trail": robot_trail
        })
        
        await asyncio.sleep(0.08)  # Faster updates for smoother movement

# Background task for periodic updates
async def periodic_updates():
    """Send periodic status updates."""
    while True:
        # Update system uptime continuously
        current_time = time.time()
        metrics["system_uptime"] = current_time - start_time
        
        # Update performance summary calculations
        if metrics["system_uptime"] > 0:
            metrics["goals_per_hour"] = (metrics["goals_completed"] / (metrics["system_uptime"] / 3600))
            metrics["replan_rate"] = (metrics["total_replans"] / (metrics["system_uptime"] / 60))
            metrics["avg_speed"] = metrics["distance_traveled"] / (metrics["system_uptime"] / 3600)  # m/h
            
            # Calculate success rate (cap at 100%)
            total_possible_goals = metrics["total_missions"] * 7  # 7 goals per demo mission
            if total_possible_goals > 0:
                success_rate = (metrics["goals_completed"] / total_possible_goals) * 100
                metrics["success_rate"] = min(success_rate, 100.0)  # Cap at 100%
        
        await broadcast_to_clients({
            "type": "status_update",
            "robot_status": robot_status,
            "metrics": metrics
        })
        await asyncio.sleep(1)  # Update every second

# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize the server and start background tasks."""
    logger.info("FastAPI server started with background tasks")
    
    # Initialize demo data
    logger.info("Demo data initialized")
    
    # Start periodic updates
    asyncio.create_task(periodic_updates())

if __name__ == "__main__":
    import uvicorn
    import os
    
    port = int(os.environ.get("PORT", 8001))
    logger.info(f"Starting FastAPI server on 0.0.0.0:{port}")
    uvicorn.run(app, host="0.0.0.0", port=port)
