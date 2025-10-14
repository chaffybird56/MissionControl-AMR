import { motion } from 'framer-motion'
import { Wifi, WifiOff, Activity } from 'lucide-react'

interface ConnectionStatusProps {
  isConnected: boolean
  robotState: string
}

export default function ConnectionStatus({ isConnected, robotState }: ConnectionStatusProps) {
  const getStatusColor = (connected: boolean, state: string) => {
    if (!connected) return 'text-red-600 bg-red-100'
    if (state === 'running') return 'text-green-600 bg-green-100'
    if (state === 'idle') return 'text-blue-600 bg-blue-100'
    if (state === 'paused') return 'text-yellow-600 bg-yellow-100'
    if (state === 'failed') return 'text-red-600 bg-red-100'
    return 'text-gray-600 bg-gray-100'
  }

  const getStatusText = (connected: boolean, state: string) => {
    if (!connected) return 'Disconnected'
    if (state === 'running') return 'Active Mission'
    if (state === 'idle') return 'Standby'
    if (state === 'paused') return 'Paused'
    if (state === 'failed') return 'Error'
    return 'Unknown'
  }

  const getStatusIcon = (connected: boolean) => {
    return connected ? Wifi : WifiOff
  }

  const StatusIcon = getStatusIcon(isConnected)

  return (
    <motion.div
      initial={{ opacity: 0, scale: 0.9 }}
      animate={{ opacity: 1, scale: 1 }}
      className="flex items-center space-x-3"
    >
      <div className="flex items-center space-x-2">
        <StatusIcon className="h-5 w-5" />
        {isConnected && robotState === 'running' && (
          <motion.div
            animate={{ scale: [1, 1.2, 1] }}
            transition={{ duration: 1, repeat: Infinity }}
          >
            <Activity className="h-4 w-4 text-green-600" />
          </motion.div>
        )}
      </div>
      
      <div className="text-right">
        <div className={`inline-flex items-center px-3 py-1 rounded-full text-sm font-medium ${getStatusColor(isConnected, robotState)}`}>
          {getStatusText(isConnected, robotState)}
        </div>
        <div className="text-xs text-gray-500 mt-1">
          {isConnected ? 'WebSocket Connected' : 'Connection Lost'}
        </div>
      </div>
    </motion.div>
  )
}

