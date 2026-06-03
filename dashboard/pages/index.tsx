import { useState, useEffect } from 'react'
import Head from 'next/head'
import { motion } from 'framer-motion'
import MissionPanel from '../components/MissionPanel'
import RobotStatus from '../components/RobotStatus'
import MapView from '../components/MapView'
import MetricsPanel from '../components/MetricsPanel'
import ConnectionStatus from '../components/ConnectionStatus'

export interface RobotPose {
  x: number
  y: number
  theta: number
}

export interface RobotVelocity {
  linear_x: number
  angular_z: number
}

export interface RobotData {
  pose: RobotPose
  velocity: RobotVelocity
  battery_level: number
  mission_state: string
  goal_state: string
  timestamp: number
}

export interface Metrics {
  goals_completed: number
  total_replans: number
  distance_traveled: number
  success_rate: number
  system_uptime: number
}

export default function Home() {
  const [robotData, setRobotData] = useState<RobotData>({
    pose: { x: 0, y: 0, theta: 0 },
    velocity: { linear_x: 0, angular_z: 0 },
    battery_level: 100,
    mission_state: 'idle',
    goal_state: 'none',
    timestamp: Date.now()
  })
  
  const [metrics, setMetrics] = useState<Metrics>({
    goals_completed: 0,
    total_replans: 0,
    distance_traveled: 0,
    success_rate: 0,
    system_uptime: 0
  })
  
  const [isConnected, setIsConnected] = useState(false)
  const [currentMission, setCurrentMission] = useState<string | null>(null)

  return (
    <>
      <Head>
        <title>ROS2 AMR Dashboard</title>
        <meta name="description" content="Autonomous Mobile Robot Control Dashboard" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className="min-h-screen bg-gray-50">
        {/* Header */}
        <header className="bg-white shadow-sm border-b border-gray-200">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="flex justify-between items-center py-4">
              <div className="flex items-center space-x-4">
                <motion.div
                  initial={{ scale: 0 }}
                  animate={{ scale: 1 }}
                  transition={{ type: "spring", stiffness: 200 }}
                  className="w-10 h-10 bg-primary-600 rounded-lg flex items-center justify-center"
                >
                  <span className="text-white font-bold text-lg">🤖</span>
                </motion.div>
                <div>
                  <h1 className="text-2xl font-bold text-gray-900">ROS2 AMR Dashboard</h1>
                  <p className="text-sm text-gray-500">Autonomous Mobile Robot Control Center</p>
                </div>
              </div>
              
              <div className="flex items-center space-x-4">
                <ConnectionStatus 
                  isConnected={isConnected}
                  robotState={robotData.mission_state}
                />
              </div>
            </div>
          </div>
        </header>

        {/* Main Content */}
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
          {/* Status Cards */}
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6 mb-8">
            <RobotStatus 
              robotData={robotData}
              setRobotData={setRobotData}
              setIsConnected={setIsConnected}
              setMetrics={setMetrics}
            />
          </div>

          {/* Main Dashboard Grid */}
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
            {/* Mission Panel - Left Column */}
            <div className="lg:col-span-1">
              <MissionPanel 
                currentMission={currentMission}
                setCurrentMission={setCurrentMission}
                robotState={robotData.mission_state}
              />
            </div>

            {/* Map View - Right Column */}
            <div className="lg:col-span-2">
              <MapView 
                robotPose={robotData.pose}
                robotVelocity={robotData.velocity}
                missionState={robotData.mission_state}
                currentMission={currentMission}
              />
            </div>
          </div>

          {/* Metrics Panel */}
          <div className="grid grid-cols-1">
            <MetricsPanel metrics={metrics} />
          </div>
        </div>

        {/* Footer */}
        <footer className="bg-white border-t border-gray-200 mt-12">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-6">
            <div className="flex justify-between items-center">
              <div className="text-sm text-gray-500">
                ROS2 AMR System v1.0.0 | Built with Next.js & Tailwind CSS
              </div>
              <div className="text-sm text-gray-500">
                Uptime: {Math.floor(metrics.system_uptime / 60)}m {Math.floor(metrics.system_uptime % 60)}s
              </div>
            </div>
          </div>
        </footer>
      </main>
    </>
  )
}
