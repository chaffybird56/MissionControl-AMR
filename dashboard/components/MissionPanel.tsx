import { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import { Play, Square, Pause, MapPin, Plus, Trash2, RotateCcw } from 'lucide-react'
import toast from 'react-hot-toast'

interface MissionPanelProps {
  currentMission: string | null
  setCurrentMission: (mission: string | null) => void
  robotState: string
}

interface Goal {
  x: number
  y: number
  theta: number
}

interface Mission {
  id: string
  name: string
  description: string
  goals: Goal[]
  status: string
  created_at: string
}

export default function MissionPanel({ currentMission, setCurrentMission, robotState }: MissionPanelProps) {
  const [missions, setMissions] = useState<Mission[]>([])
  const [isLoading, setIsLoading] = useState(false)
  const [newGoal, setNewGoal] = useState({ x: 0, y: 0, theta: 0 })
  const [newMissionName, setNewMissionName] = useState('')

  // Fetch missions on component mount
  useEffect(() => {
    fetchMissions()
  }, [])

  const fetchMissions = async () => {
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/missions`)
      if (response.ok) {
        const data = await response.json()
        setMissions(data.missions || [])
      }
    } catch (error) {
      console.error('Failed to fetch missions:', error)
    }
  }

  const startMission = async (missionId: string) => {
    setIsLoading(true)
    try {
      const response = await fetch(
        `${process.env.API_URL || 'http://localhost:8002'}/api/mission/${missionId}/start`,
        { method: 'POST' }
      )
      
      if (response.ok) {
        setCurrentMission(missionId)
        toast.success('Mission started successfully!')
      } else {
        toast.error('Failed to start mission')
      }
    } catch (error) {
      toast.error('Error starting mission')
      console.error('Start mission error:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const stopMission = async () => {
    setIsLoading(true)
    try {
      const response = await fetch(
        `${process.env.API_URL || 'http://localhost:8002'}/api/mission/stop`,
        { method: 'POST' }
      )
      
      if (response.ok) {
        setCurrentMission(null)
        toast.success('Mission stopped successfully!')
        fetchMissions() // Refresh missions to update status
      } else {
        toast.error('Failed to stop mission')
      }
    } catch (error) {
      toast.error('Error stopping mission')
      console.error('Stop mission error:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const resetRobot = async () => {
    setIsLoading(true)
    try {
      const response = await fetch(
        `${process.env.API_URL || 'http://localhost:8002'}/api/robot/reset`,
        { method: 'POST' }
      )
      
      if (response.ok) {
        setCurrentMission(null)
        toast.success('Robot reset to origin (0,0) and metrics cleared!')
        fetchMissions() // Refresh missions to update status
      } else {
        toast.error('Failed to reset robot')
      }
    } catch (error) {
      toast.error('Error resetting robot')
      console.error('Reset robot error:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const startGenericMission = async () => {
    setIsLoading(true)
    try {
      const response = await fetch(`${process.env.API_URL || 'http://localhost:8002'}/api/mission/start`, {
        method: 'POST'
      })
      
      if (response.ok) {
        const data = await response.json()
        setCurrentMission(data.mission_id)
        toast.success(`Mission started with ${data.goals} goals`)
        await fetchMissions() // Refresh missions list
      } else {
        const error = await response.json()
        toast.error(`Failed to start mission: ${error.detail || 'Unknown error'}`)
      }
    } catch (error) {
      console.error('Error starting mission:', error)
      toast.error('Failed to start mission')
    } finally {
      setIsLoading(false)
    }
  }

  const createMission = async () => {
    if (!newMissionName.trim()) {
      toast.error('Please enter a mission name')
      return
    }

    setIsLoading(true)
    try {
      const response = await fetch(
        `${process.env.API_URL || 'http://localhost:8002'}/api/mission`,
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            name: newMissionName,
            goals: [newGoal],
            priority: 1
          })
        }
      )
      
      if (response.ok) {
        toast.success('Mission created successfully!')
        setNewMissionName('')
        setNewGoal({ x: 0, y: 0, theta: 0 })
        fetchMissions()
      } else {
        toast.error('Failed to create mission')
      }
    } catch (error) {
      toast.error('Error creating mission')
      console.error('Create mission error:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const getMissionStatusColor = (state: string) => {
    switch (state) {
      case 'running':
        return 'bg-green-100 text-green-800'
      case 'idle':
        return 'bg-gray-100 text-gray-800'
      case 'paused':
        return 'bg-yellow-100 text-yellow-800'
      case 'failed':
        return 'bg-red-100 text-red-800'
      default:
        return 'bg-gray-100 text-gray-800'
    }
  }

  const getMissionStatusText = (state: string) => {
    switch (state) {
      case 'running':
        return 'Running'
      case 'idle':
        return 'Idle'
      case 'paused':
        return 'Paused'
      case 'failed':
        return 'Failed'
      default:
        return 'Unknown'
    }
  }

  return (
    <div className="card">
      <div className="flex items-center justify-between mb-6">
        <h2 className="text-xl font-semibold text-gray-900">Mission Control</h2>
        <div className="flex items-center space-x-3">
          <span className={`status-indicator ${getMissionStatusColor(robotState)}`}>
            {getMissionStatusText(robotState)}
          </span>
          <button
            onClick={resetRobot}
            disabled={isLoading}
            className="btn-secondary text-sm flex items-center space-x-1"
            title="Reset robot to origin (0,0) and clear metrics"
          >
            <RotateCcw className="h-4 w-4" />
            <span>Reset</span>
          </button>
        </div>
      </div>

      {/* Generic Start Button */}
      <div className="bg-gray-50 rounded-lg p-4 mb-6">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="font-medium text-gray-900">Start Mission</h3>
            <p className="text-sm text-gray-500">Use goals placed on the map</p>
          </div>
          <button
            onClick={startGenericMission}
            disabled={isLoading || robotState === 'running'}
            className="btn-primary flex items-center space-x-2"
          >
            <Play className="h-4 w-4" />
            <span>Start</span>
          </button>
        </div>
      </div>

      {/* Mission List */}
      <div className="space-y-4 mb-6">
        {/* Demo Mission */}
        <div className="bg-blue-50 rounded-lg p-4">
          <div className="flex items-center justify-between">
            <div>
              <h3 className="font-medium text-gray-900">Demo Navigation Mission</h3>
              <p className="text-sm text-gray-500">Automated demo with random obstacles</p>
            </div>
            <button
              onClick={() => startMission("demo_mission_001")}
              disabled={isLoading || robotState === 'running'}
              className="btn-secondary flex items-center space-x-2"
            >
              <Play className="h-4 w-4" />
              <span>Demo</span>
            </button>
          </div>
        </div>

        {missions.filter(m => m.id !== "demo_mission_001").length === 0 ? (
          <div className="text-center py-8 text-gray-500">
            <MapPin className="mx-auto h-12 w-12 mb-4 opacity-50" />
            <p>No custom missions available</p>
            <p className="text-sm">Create a new mission or use the map goals</p>
          </div>
        ) : (
          missions.filter(m => m.id !== "demo_mission_001").map((mission) => (
            <motion.div
              key={mission.id}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              className={`border rounded-lg p-4 ${
                mission.status === 'running' ? 'border-primary-500 bg-primary-50' : 'border-gray-200'
              }`}
            >
              <div className="flex items-center justify-between mb-2">
                <h3 className="font-medium text-gray-900">{mission.name}</h3>
                <span className="text-sm text-gray-500">
                  {mission.goals.length} goal{mission.goals.length !== 1 ? 's' : ''}
                </span>
              </div>
              
              <div className="flex space-x-2">
                {mission.status !== 'running' ? (
                  <button
                    onClick={() => startMission(mission.id)}
                    disabled={isLoading || robotState === 'running'}
                    className="btn-primary text-sm flex items-center space-x-1"
                  >
                    <Play className="h-4 w-4" />
                    <span>Start</span>
                  </button>
                ) : (
                  <button
                    onClick={stopMission}
                    disabled={isLoading}
                    className="btn-danger text-sm flex items-center space-x-1"
                  >
                    <Square className="h-4 w-4" />
                    <span>Stop</span>
                  </button>
                )}
              </div>
            </motion.div>
          ))
        )}
      </div>

      {/* Create New Mission */}
      <div className="border-t pt-6">
        <h3 className="font-medium text-gray-900 mb-4">Create New Mission</h3>
        
        <div className="space-y-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Mission Name
            </label>
            <input
              type="text"
              value={newMissionName}
              onChange={(e) => setNewMissionName(e.target.value)}
              placeholder="Enter mission name"
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500 focus:border-transparent"
            />
          </div>
          
          <div className="grid grid-cols-3 gap-2">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">X</label>
              <input
                type="number"
                step="0.1"
                value={newGoal.x}
                onChange={(e) => setNewGoal({ ...newGoal, x: parseFloat(e.target.value) || 0 })}
                className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">Y</label>
              <input
                type="number"
                step="0.1"
                value={newGoal.y}
                onChange={(e) => setNewGoal({ ...newGoal, y: parseFloat(e.target.value) || 0 })}
                className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">θ</label>
              <input
                type="number"
                step="0.1"
                value={newGoal.theta}
                onChange={(e) => setNewGoal({ ...newGoal, theta: parseFloat(e.target.value) || 0 })}
                className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-primary-500"
              />
            </div>
          </div>
          
          <button
            onClick={createMission}
            disabled={isLoading || !newMissionName.trim()}
            className="btn-primary w-full flex items-center justify-center space-x-2"
          >
            <Plus className="h-4 w-4" />
            <span>Create Mission</span>
          </button>
        </div>
      </div>
    </div>
  )
}
