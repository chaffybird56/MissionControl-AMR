import { useState, useEffect, useRef } from 'react'
import { motion } from 'framer-motion'
import { Map, Navigation, Target, ZoomIn, ZoomOut, RotateCcw } from 'lucide-react'
import { RobotPose, RobotVelocity } from '../pages/index'

interface MapViewProps {
  robotPose: RobotPose
  robotVelocity: RobotVelocity
  missionState: string
  currentMission: string | null
}

interface Goal {
  x: number
  y: number
  theta: number
}

export default function MapView({ robotPose, robotVelocity, missionState, currentMission }: MapViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [goals, setGoals] = useState<Goal[]>([])
  const [obstacles, setObstacles] = useState<any[]>([])
  const [trail, setTrail] = useState<RobotPose[]>([])
  const [mapSize, setMapSize] = useState({ width: 600, height: 450 }) // Bigger map
  const [mapScale, setMapScale] = useState(25) // pixels per meter (increased from 20)
  const [zoom, setZoom] = useState(1.0) // Zoom factor
  const [missionGoals, setMissionGoals] = useState<Goal[]>([])
  const [missionObstacles, setMissionObstacles] = useState<any[]>([])

  // Fetch map elements from server
  useEffect(() => {
    fetchMapElements()
  }, [])

  // WebSocket connection for real-time updates
  useEffect(() => {
    const wsUrl = process.env.WS_URL || 'ws://localhost:8002/ws'
    const websocket = new WebSocket(wsUrl)

    websocket.onopen = () => {
      console.log('MapView WebSocket connected')
    }

    websocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        
        if (data.type === 'mission_started') {
          // Handle mission start - set mission obstacles and goals
          if (data.mission) {
            setMissionGoals(data.mission.goals || [])
            setMissionObstacles(data.mission.obstacles || [])
          }
        } else if (data.type === 'map_update') {
          // Handle map updates
          if (data.goals) setGoals(data.goals)
          if (data.obstacles) setObstacles(data.obstacles)
          if (data.trail) setTrail(data.trail)
        } else if (data.type === 'robot_reset') {
          // Handle robot reset - clear mission data
          setMissionGoals([])
          setMissionObstacles([])
          if (data.goals) setGoals(data.goals)
          if (data.obstacles) setObstacles(data.obstacles)
          if (data.trail) setTrail(data.trail)
        }
      } catch (error) {
        console.error('MapView WebSocket message parsing error:', error)
      }
    }

    websocket.onerror = (error) => {
      console.error('MapView WebSocket error:', error)
    }

    websocket.onclose = () => {
      console.log('MapView WebSocket disconnected')
    }

    return () => {
      websocket.close()
    }
  }, [])

  const fetchMapElements = async () => {
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/map/elements`)
      if (response.ok) {
        const data = await response.json()
        setGoals(data.goals || [])
        setObstacles(data.obstacles || [])
        setTrail(data.trail || [])
      }
    } catch (error) {
      console.error('Failed to fetch map elements:', error)
    }
  }

  // Fetch mission goals when a mission is active
  useEffect(() => {
    if (currentMission) {
      fetchMissionGoals(currentMission)
    } else {
      setMissionGoals([])
      setMissionObstacles([])
    }
  }, [currentMission])

  const fetchMissionGoals = async (missionId: string) => {
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/missions`)
      if (response.ok) {
        const data = await response.json()
        const mission = data.missions.find((m: any) => m.id === missionId)
        if (mission) {
          setMissionGoals(mission.goals || [])
          setMissionObstacles(mission.obstacles || [])
        }
      }
    } catch (error) {
      console.error('Failed to fetch mission goals:', error)
    }
  }

  // Draw the map
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height)

    // Set up coordinate system (origin at center, Y-axis flipped)
    ctx.save()
    ctx.translate(canvas.width / 2, canvas.height / 2)
    ctx.scale(zoom, -zoom) // Flip Y-axis to match ROS convention and apply zoom

    // Draw grid
    drawGrid(ctx, canvas.width, canvas.height)

    // Draw obstacles (simplified representation)
    // Draw obstacles (mission + user)
    drawObstacles(ctx, [...missionObstacles, ...obstacles])

    // Draw robot trail
    drawTrail(ctx)
    
    // Draw goals (mission + user)
    drawGoals(ctx, [...missionGoals, ...goals])

    // Draw robot
    drawRobot(ctx, robotPose)

    ctx.restore()
  }, [robotPose, trail, goals, obstacles, missionGoals, missionObstacles, mapSize, zoom])

  const drawGrid = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
    ctx.strokeStyle = '#e5e7eb'
    ctx.lineWidth = 1

    const gridSpacing = mapScale
    const halfWidth = width / 2
    const halfHeight = height / 2

    // Vertical lines
    for (let x = -halfWidth; x <= halfWidth; x += gridSpacing) {
      ctx.beginPath()
      ctx.moveTo(x, -halfHeight)
      ctx.lineTo(x, halfHeight)
      ctx.stroke()
    }

    // Horizontal lines
    for (let y = -halfHeight; y <= halfHeight; y += gridSpacing) {
      ctx.beginPath()
      ctx.moveTo(-halfWidth, y)
      ctx.lineTo(halfWidth, y)
      ctx.stroke()
    }

    // Draw axes
    ctx.strokeStyle = '#6b7280'
    ctx.lineWidth = 2

    // X-axis
    ctx.beginPath()
    ctx.moveTo(-halfWidth, 0)
    ctx.lineTo(halfWidth, 0)
    ctx.stroke()

    // Y-axis
    ctx.beginPath()
    ctx.moveTo(0, -halfHeight)
    ctx.lineTo(0, halfHeight)
    ctx.stroke()

    // Origin marker
    ctx.fillStyle = '#6b7280'
    ctx.beginPath()
    ctx.arc(0, 0, 3, 0, 2 * Math.PI)
    ctx.fill()
  }

  const drawObstacles = (ctx: CanvasRenderingContext2D, allObstacles: any[]) => {
    // Draw professional obstacles with state-of-the-art design
    allObstacles.forEach(obstacle => {
      const x = obstacle.x * mapScale * zoom
      const y = -obstacle.y * mapScale * zoom
      const size = obstacle.size * mapScale * zoom
      
      // Professional shadow first
      ctx.shadowColor = 'rgba(0, 0, 0, 0.25)'
      ctx.shadowBlur = 12
      ctx.shadowOffsetX = 4
      ctx.shadowOffsetY = 4
      
      // Professional gradient - deep red with metallic finish
      const gradient = ctx.createLinearGradient(x - size, y - size, x + size, y + size)
      gradient.addColorStop(0, '#dc2626')  // Deep red
      gradient.addColorStop(0.3, '#b91c1c') // Darker red
      gradient.addColorStop(0.7, '#991b1b') // Even darker
      gradient.addColorStop(1, '#7f1d1d')  // Darkest red
      
      ctx.fillStyle = gradient
      
      // Professional rounded rectangle with subtle border
      const cornerRadius = 6
      ctx.beginPath()
      ctx.roundRect(x - size, y - size, size * 2, size * 2, cornerRadius)
      ctx.fill()
      
      // Professional border
      ctx.strokeStyle = '#450a0a'
      ctx.lineWidth = 2
      ctx.stroke()
      
      // Add subtle inner highlight
      const highlightGradient = ctx.createLinearGradient(x - size, y - size, x + size, y + size)
      highlightGradient.addColorStop(0, 'rgba(255, 255, 255, 0.2)')
      highlightGradient.addColorStop(1, 'rgba(255, 255, 255, 0)')
      
      ctx.fillStyle = highlightGradient
      ctx.beginPath()
      ctx.roundRect(x - size + 2, y - size + 2, size * 2 - 4, size * 2 - 4, cornerRadius - 2)
      ctx.fill()
      
      // Reset shadow
      ctx.shadowBlur = 0
      ctx.shadowOffsetX = 0
      ctx.shadowOffsetY = 0
    })
  }

  const drawTrail = (ctx: CanvasRenderingContext2D) => {
    if (trail.length < 2) return

    // Draw a puffy, gradient trail
    trail.forEach((point, index) => {
      const x = point.x * mapScale * zoom
      const y = -point.y * mapScale * zoom
      
      // Fade out older trail points and make them smaller
      const alpha = Math.max(0.1, 1 - (index / trail.length))
      const size = Math.max(2, 6 - (index / trail.length) * 4)
      
      // Create gradient for each trail point
      const gradient = ctx.createRadialGradient(x, y, 0, x, y, size)
      gradient.addColorStop(0, `rgba(59, 130, 246, ${alpha})`)
      gradient.addColorStop(1, `rgba(59, 130, 246, ${alpha * 0.2})`)
      
      ctx.fillStyle = gradient
      ctx.beginPath()
      ctx.arc(x, y, size, 0, 2 * Math.PI)
      ctx.fill()
    })
  }

  const drawGoals = (ctx: CanvasRenderingContext2D, allGoals: Goal[]) => {
    // Draw professional mission goals with state-of-the-art design
    allGoals.forEach((goal, index) => {
      const x = goal.x * mapScale * zoom
      const y = -goal.y * mapScale * zoom
      
      // Professional shadow first
      ctx.shadowColor = 'rgba(0, 0, 0, 0.2)'
      ctx.shadowBlur = 10
      ctx.shadowOffsetX = 3
      ctx.shadowOffsetY = 3
      
      // Professional gradient - blue with metallic finish
      const gradient = ctx.createLinearGradient(x - 14, y - 14, x + 14, y + 14)
      gradient.addColorStop(0, '#3b82f6')  // Blue
      gradient.addColorStop(0.3, '#2563eb') // Darker blue
      gradient.addColorStop(0.7, '#1d4ed8') // Even darker
      gradient.addColorStop(1, '#1e40af')  // Darkest blue
      
      ctx.fillStyle = gradient
      ctx.beginPath()
      ctx.arc(x, y, 14, 0, 2 * Math.PI)
      ctx.fill()
      
      // Professional border
      ctx.strokeStyle = '#1e3a8a'
      ctx.lineWidth = 2
      ctx.stroke()
      
      // Add subtle inner highlight
      const highlightGradient = ctx.createRadialGradient(x - 5, y - 5, 0, x + 5, y + 5, 10)
      highlightGradient.addColorStop(0, 'rgba(255, 255, 255, 0.3)')
      highlightGradient.addColorStop(1, 'rgba(255, 255, 255, 0)')
      
      ctx.fillStyle = highlightGradient
      ctx.beginPath()
      ctx.arc(x, y, 10, 0, 2 * Math.PI)
      ctx.fill()

      // Professional goal number with high contrast - NO ARROWS - FIXED ORIENTATION
      ctx.fillStyle = '#ffffff'
      ctx.font = 'bold 20px Arial'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.strokeStyle = '#1e3a8a'
      ctx.lineWidth = 4
      
      // Save context for text transformation
      ctx.save()
      ctx.translate(x, y)
      ctx.scale(1, -1)  // Flip Y-axis to correct text orientation
      ctx.strokeText((index + 1).toString(), 0, 0)
      ctx.fillText((index + 1).toString(), 0, 0)
      ctx.restore()
      
      // Reset shadow
      ctx.shadowBlur = 0
      ctx.shadowOffsetX = 0
      ctx.shadowOffsetY = 0
    })

    // Draw user-added goals (smaller, different color)
    goals.forEach((goal, index) => {
      // User goal marker (smaller, different color)
      ctx.fillStyle = '#10b981'
      ctx.beginPath()
      ctx.arc(goal.x * mapScale, goal.y * mapScale, 6, 0, 2 * Math.PI)
      ctx.fill()

      // Goal number
      ctx.fillStyle = 'white'
      ctx.font = '12px Arial'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText((index + 1).toString(), goal.x * mapScale, goal.y * mapScale)

      // Goal orientation arrow
      const arrowLength = 15
      const endX = goal.x * mapScale + arrowLength * Math.cos(goal.theta)
      const endY = goal.y * mapScale + arrowLength * Math.sin(goal.theta)
      
      ctx.strokeStyle = '#10b981'
      ctx.lineWidth = 3
      ctx.beginPath()
      ctx.moveTo(goal.x * mapScale, goal.y * mapScale)
      ctx.lineTo(endX, endY)
      ctx.stroke()

      // Arrow head
      const headLength = 5
      const angle = goal.theta
      ctx.beginPath()
      ctx.moveTo(endX, endY)
      ctx.lineTo(
        endX - headLength * Math.cos(angle - Math.PI / 6),
        endY - headLength * Math.sin(angle - Math.PI / 6)
      )
      ctx.moveTo(endX, endY)
      ctx.lineTo(
        endX - headLength * Math.cos(angle + Math.PI / 6),
        endY - headLength * Math.sin(angle + Math.PI / 6)
      )
      ctx.stroke()
    })
  }

  const drawRobot = (ctx: CanvasRenderingContext2D, pose: RobotPose) => {
    const x = pose.x * mapScale * zoom
    const y = -pose.y * mapScale * zoom
    const theta = pose.theta

    // Save context for rotation
    ctx.save()
    ctx.translate(x, y)
    ctx.rotate(theta)

    const isMoving = (pose.velocity?.linear_x || 0) > 0 || (pose.velocity?.angular_z || 0) > 0
    
    // Professional robot design - sleek and modern
    const robotSize = 18
    
    // Outer glow ring when moving - professional effect
    if (isMoving) {
      const glowGradient = ctx.createRadialGradient(0, 0, robotSize, 0, 0, robotSize + 12)
      glowGradient.addColorStop(0, 'rgba(16, 185, 129, 0.2)')
      glowGradient.addColorStop(1, 'rgba(16, 185, 129, 0)')
      ctx.fillStyle = glowGradient
      ctx.beginPath()
      ctx.arc(0, 0, robotSize + 12, 0, 2 * Math.PI)
      ctx.fill()
    }
    
    // Main body - professional sleek design with metallic gradient
    const bodyGradient = ctx.createLinearGradient(-robotSize, -robotSize, robotSize, robotSize)
    bodyGradient.addColorStop(0, '#14b8a6')  // Bright teal
    bodyGradient.addColorStop(0.3, '#0d9488') // Darker teal
    bodyGradient.addColorStop(0.7, '#0f766e') // Even darker
    bodyGradient.addColorStop(1, '#064e3b')  // Darkest teal
    
    ctx.fillStyle = bodyGradient
    ctx.beginPath()
    // Draw sleek rounded rectangle (professional shape)
    const cornerRadius = 6
    ctx.roundRect(-robotSize, -robotSize/2, robotSize * 2, robotSize, cornerRadius)
    ctx.fill()
    
    // Professional metallic border
    ctx.strokeStyle = '#064e3b'
    ctx.lineWidth = 2
    ctx.stroke()

    // Add subtle inner highlight for metallic effect
    const highlightGradient = ctx.createLinearGradient(-robotSize, -robotSize/2, robotSize, robotSize/2)
    highlightGradient.addColorStop(0, 'rgba(255, 255, 255, 0.3)')
    highlightGradient.addColorStop(1, 'rgba(255, 255, 255, 0)')
    ctx.fillStyle = highlightGradient
    ctx.beginPath()
    ctx.roundRect(-robotSize + 2, -robotSize/2 + 2, robotSize * 2 - 4, robotSize - 4, cornerRadius - 2)
    ctx.fill()

    // Professional status indicator - single LED
    const statusColor = isMoving ? '#10b981' : '#6b7280'
    ctx.fillStyle = statusColor
    ctx.beginPath()
    ctx.arc(0, -robotSize/2 + 4, 4, 0, 2 * Math.PI)
    ctx.fill()
    
    // Subtle glow for status LED
    ctx.shadowColor = statusColor
    ctx.shadowBlur = 8
    ctx.fill()
    ctx.shadowBlur = 0

    // Professional direction indicator - sleek arrow
    ctx.fillStyle = '#fbbf24'
    ctx.beginPath()
    ctx.moveTo(robotSize - 3, -6)
    ctx.lineTo(robotSize + 8, 0)
    ctx.lineTo(robotSize - 3, 6)
    ctx.closePath()
    ctx.fill()

    // Professional wheels - sleek design
    ctx.fillStyle = '#1f2937'
    ctx.fillRect(-robotSize + 3, -robotSize/2 + 2, 4, 8)
    ctx.fillRect(robotSize - 7, -robotSize/2 + 2, 4, 8)

    ctx.restore()
  }

  // Clear functions removed - now handled by universal reset button

  const zoomIn = () => {
    setZoom(prev => Math.min(prev * 1.2, 3.0))
  }

  const zoomOut = () => {
    setZoom(prev => Math.max(prev / 1.2, 0.3))
  }

  const resetZoom = () => {
    setZoom(1.0)
  }

  // Handle canvas clicks for adding goals and obstacles
  const handleCanvasClick = async (event: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return

    const rect = canvas.getBoundingClientRect()
    const x = event.clientX - rect.left
    const y = event.clientY - rect.top

    // Convert canvas coordinates to world coordinates
    const worldX = (x - mapSize.width / 2) / (mapScale * zoom)
    const worldY = -(y - mapSize.height / 2) / (mapScale * zoom)

    // Add goal on left click
    if (event.button === 0) {
      await addGoal(worldX, worldY, 0)
    }
    // Add obstacle on right click
    else if (event.button === 2) {
      event.preventDefault()
      await addObstacle(worldX, worldY)
    }
  }

  const addGoal = async (x: number, y: number, theta: number) => {
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/map/goals`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, theta })
      })
      if (response.ok) {
        const data = await response.json()
        setGoals(data.goals || [])
      }
    } catch (error) {
      console.error('Failed to add goal:', error)
    }
  }

  const addObstacle = async (x: number, y: number) => {
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/map/obstacles`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, type: 'rectangle', size: 0.5 })
      })
      if (response.ok) {
        const data = await response.json()
        setObstacles(data.obstacles || [])
      }
    } catch (error) {
      console.error('Failed to add obstacle:', error)
    }
  }

  return (
    <div className="card">
      <div className="flex items-center justify-between mb-6">
        <div className="flex items-center space-x-3">
          <Map className="h-5 w-5 text-primary-600" />
          <h2 className="text-xl font-semibold text-gray-900">Map View</h2>
        </div>
        <div className="flex space-x-2">
          <button
            onClick={zoomOut}
            className="btn-secondary text-sm flex items-center space-x-1"
            title="Zoom out"
          >
            <ZoomOut className="h-4 w-4" />
          </button>
          <button
            onClick={resetZoom}
            className="btn-secondary text-sm flex items-center space-x-1"
            title="Reset zoom"
          >
            <RotateCcw className="h-4 w-4" />
            <span>{Math.round(zoom * 100)}%</span>
          </button>
          <button
            onClick={zoomIn}
            className="btn-secondary text-sm flex items-center space-x-1"
            title="Zoom in"
          >
            <ZoomIn className="h-4 w-4" />
          </button>
        </div>
      </div>

      {/* Map Canvas */}
      <div className="relative">
        <canvas
          ref={canvasRef}
          width={mapSize.width}
          height={mapSize.height}
          onClick={handleCanvasClick}
          className="border border-gray-300 rounded-lg cursor-crosshair bg-white"
        />
        
        {/* Map overlay info */}
        <div className="absolute top-2 left-2 bg-white/90 backdrop-blur-sm rounded-lg p-2 text-xs text-gray-600">
          <div>Click to add goals</div>
          <div>Scale: 1m = {Math.round(mapScale * zoom)}px</div>
          <div>Zoom: {Math.round(zoom * 100)}%</div>
        </div>

        {/* Legend */}
        <div className="absolute top-2 right-2 bg-white/90 backdrop-blur-sm rounded-lg p-2 text-xs text-gray-600">
          <div className="flex items-center space-x-2 mb-1">
            <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
            <span>Robot</span>
          </div>
          <div className="flex items-center space-x-2 mb-1">
            <div className="w-3 h-3 bg-blue-600 rounded-full"></div>
            <span>Goals</span>
          </div>
          <div className="flex items-center space-x-2 mb-1">
            <div className="w-3 h-3 bg-red-500 rounded-full"></div>
            <span>Obstacles</span>
          </div>
          <div className="flex items-center space-x-2">
            <div className="w-3 h-3 bg-blue-300 rounded-full"></div>
            <span>Trail</span>
          </div>
        </div>
      </div>

      {/* Robot Info */}
      <div className="mt-4 grid grid-cols-2 gap-4 text-sm">
        <div className="bg-gray-50 rounded-lg p-3">
          <div className="flex items-center space-x-2 mb-2">
            <Navigation className="h-4 w-4 text-primary-600" />
            <span className="font-medium text-gray-700">Position</span>
          </div>
          <div className="text-gray-600">
            <div>X: {robotPose.x.toFixed(2)}m</div>
            <div>Y: {robotPose.y.toFixed(2)}m</div>
            <div>θ: {(robotPose.theta * 180 / Math.PI).toFixed(1)}°</div>
          </div>
        </div>

        <div className="bg-gray-50 rounded-lg p-3">
          <div className="flex items-center space-x-2 mb-2">
            <Target className="h-4 w-4 text-primary-600" />
            <span className="font-medium text-gray-700">Velocity</span>
          </div>
          <div className="text-gray-600">
            <div>Linear: {robotVelocity.linear_x.toFixed(2)} m/s</div>
            <div>Angular: {robotVelocity.angular_z.toFixed(2)} rad/s</div>
            <div>Speed: {Math.sqrt(robotVelocity.linear_x ** 2 + robotVelocity.angular_z ** 2).toFixed(2)}</div>
          </div>
        </div>
      </div>

      {/* Mission Goals List */}
      {missionGoals.length > 0 && (
        <div className="mt-4">
          <h3 className="font-medium text-gray-900 mb-2">Mission Goals ({missionGoals.length})</h3>
          <div className="space-y-1 max-h-32 overflow-y-auto">
            {missionGoals.map((goal, index) => (
              <div key={index} className="flex justify-between items-center text-sm bg-blue-50 rounded p-2">
                <span className="text-gray-700">
                  Goal {index + 1}: ({goal.x.toFixed(1)}, {goal.y.toFixed(1)}, {(goal.theta * 180 / Math.PI).toFixed(1)}°)
                </span>
                <div className="w-2 h-2 bg-blue-600 rounded-full"></div>
              </div>
            ))}
          </div>
        </div>
      )}

    </div>
  )
}
