import { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import { Battery, Wifi, WifiOff, MapPin, Gauge, RotateCcw } from 'lucide-react'
import { RobotData, Metrics } from '../pages/index'

interface RobotStatusProps {
  robotData: RobotData
  setRobotData: (data: RobotData) => void
  setIsConnected: (connected: boolean) => void
  setMetrics: (metrics: Metrics) => void
}

export default function RobotStatus({ robotData, setRobotData, setIsConnected, setMetrics }: RobotStatusProps) {
  const [ws, setWs] = useState<WebSocket | null>(null)

  useEffect(() => {
    // WebSocket connection
    const connectWebSocket = () => {
      const wsUrl = process.env.WS_URL || 'ws://localhost:8002/ws'
      const websocket = new WebSocket(wsUrl)

      websocket.onopen = () => {
        console.log('WebSocket connected')
        setIsConnected(true)
        setWs(websocket)
      }

      websocket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data)
          
          if (data.type === 'status_update') {
            if (data.robot_status) {
              setRobotData({
                pose: data.robot_status.pose || robotData.pose,
                velocity: data.robot_status.velocity || robotData.velocity,
                battery_level: data.robot_status.battery_level || robotData.battery_level,
                mission_state: data.robot_status.mission_state || robotData.mission_state,
                goal_state: data.robot_status.goal_state || robotData.goal_state,
                timestamp: data.robot_status.timestamp || Date.now()
              })
            }
            
            if (data.metrics) {
              setMetrics({
                goals_completed: data.metrics.goals_completed || 0,
                total_replans: data.metrics.total_replans || 0,
                distance_traveled: data.metrics.distance_traveled || 0,
                success_rate: data.metrics.success_rate || 0,
                system_uptime: data.metrics.system_uptime || 0
              })
            }
          }
        } catch (error) {
          console.error('WebSocket message parsing error:', error)
        }
      }

      websocket.onclose = () => {
        console.log('WebSocket disconnected')
        setIsConnected(false)
        setWs(null)
        
        // Reconnect after 3 seconds
        setTimeout(connectWebSocket, 3000)
      }

      websocket.onerror = (error) => {
        console.error('WebSocket error:', error)
        setIsConnected(false)
      }
    }

    connectWebSocket()

    // Cleanup on unmount
    return () => {
      if (ws) {
        ws.close()
      }
    }
  }, [])

  const getBatteryColor = (level: number) => {
    if (level > 50) return 'text-green-600'
    if (level > 20) return 'text-yellow-600'
    return 'text-red-600'
  }

  const getBatteryIcon = (level: number) => {
    if (level > 75) return '🟢'
    if (level > 50) return '🟡'
    if (level > 25) return '🟠'
    return '🔴'
  }

  const formatPose = (pose: { x: number; y: number; theta: number }) => {
    return `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${(pose.theta * 180 / Math.PI).toFixed(1)}°)`
  }

  const formatVelocity = (velocity: { linear_x: number; angular_z: number }) => {
    return `${velocity.linear_x.toFixed(2)} m/s, ${velocity.angular_z.toFixed(2)} rad/s`
  }

  return (
    <>
      {/* Connection Status */}
      <motion.div
        initial={{ opacity: 0, scale: 0.9 }}
        animate={{ opacity: 1, scale: 1 }}
        className="card"
      >
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-3">
            {ws && ws.readyState === WebSocket.OPEN ? (
              <Wifi className="h-5 w-5 text-green-600" />
            ) : (
              <WifiOff className="h-5 w-5 text-red-600" />
            )}
            <div>
              <p className="font-medium text-gray-900">Connection</p>
              <p className="text-sm text-gray-500">
                {ws && ws.readyState === WebSocket.OPEN ? 'Connected' : 'Disconnected'}
              </p>
            </div>
          </div>
          <div className={`w-3 h-3 rounded-full ${
            ws && ws.readyState === WebSocket.OPEN ? 'bg-green-500 animate-pulse' : 'bg-red-500'
          }`} />
        </div>
      </motion.div>

      {/* Robot Position */}
      <motion.div
        initial={{ opacity: 0, scale: 0.9 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ delay: 0.1 }}
        className="card"
      >
        <div className="flex items-center space-x-3 mb-4">
          <MapPin className="h-5 w-5 text-primary-600" />
          <h3 className="font-medium text-gray-900">Robot Position</h3>
        </div>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span className="text-sm text-gray-600">Position:</span>
            <span className="text-sm font-mono">{formatPose(robotData.pose)}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-sm text-gray-600">Velocity:</span>
            <span className="text-sm font-mono">{formatVelocity(robotData.velocity)}</span>
          </div>
        </div>
      </motion.div>

      {/* Battery Status */}
      <motion.div
        initial={{ opacity: 0, scale: 0.9 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ delay: 0.2 }}
        className="card"
      >
        <div className="flex items-center space-x-3 mb-4">
          <Battery className={`h-5 w-5 ${getBatteryColor(robotData.battery_level)}`} />
          <h3 className="font-medium text-gray-900">Battery Status</h3>
        </div>
        <div className="flex items-center space-x-3">
          <div className="flex-1">
            <div className="flex justify-between text-sm mb-1">
              <span className="text-gray-600">Level</span>
              <span className={`font-medium ${getBatteryColor(robotData.battery_level)}`}>
                {robotData.battery_level.toFixed(1)}%
              </span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div
                className={`h-2 rounded-full transition-all duration-300 ${
                  robotData.battery_level > 50 ? 'bg-green-500' :
                  robotData.battery_level > 20 ? 'bg-yellow-500' : 'bg-red-500'
                }`}
                style={{ width: `${Math.max(0, Math.min(100, robotData.battery_level))}%` }}
              />
            </div>
          </div>
          <span className="text-2xl">{getBatteryIcon(robotData.battery_level)}</span>
        </div>
      </motion.div>

      {/* Mission Status */}
      <motion.div
        initial={{ opacity: 0, scale: 0.9 }}
        animate={{ opacity: 1, scale: 1 }}
        transition={{ delay: 0.3 }}
        className="card"
      >
        <div className="flex items-center space-x-3 mb-4">
          <Gauge className="h-5 w-5 text-primary-600" />
          <h3 className="font-medium text-gray-900">Mission Status</h3>
        </div>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span className="text-sm text-gray-600">State:</span>
            <span className={`text-sm font-medium ${
              robotData.mission_state === 'running' ? 'text-green-600' :
              robotData.mission_state === 'idle' ? 'text-gray-600' :
              robotData.mission_state === 'failed' ? 'text-red-600' : 'text-yellow-600'
            }`}>
              {robotData.mission_state.charAt(0).toUpperCase() + robotData.mission_state.slice(1)}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-sm text-gray-600">Goal State:</span>
            <span className="text-sm text-gray-900">{robotData.goal_state}</span>
          </div>
        </div>
        
        {/* Robot Status Indicator */}
        <div className="mt-4 flex justify-center">
          <motion.div
            animate={robotData.mission_state === 'running' ? { rotate: 360 } : { rotate: 0 }}
            transition={{ duration: 2, repeat: robotData.mission_state === 'running' ? Infinity : 0, ease: "linear" }}
            className="w-12 h-12 bg-primary-100 rounded-full flex items-center justify-center"
          >
            <span className="text-2xl">🤖</span>
          </motion.div>
        </div>
      </motion.div>
    </>
  )
}
