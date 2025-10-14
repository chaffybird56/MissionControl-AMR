import { motion } from 'framer-motion'
import { BarChart3, Clock, Target, Route, TrendingUp } from 'lucide-react'
import { Metrics } from '../pages/index'

interface MetricsPanelProps {
  metrics: Metrics
}

export default function MetricsPanel({ metrics }: MetricsPanelProps) {
  const formatTime = (seconds: number) => {
    const hours = Math.floor(seconds / 3600)
    const minutes = Math.floor((seconds % 3600) / 60)
    const secs = Math.floor(seconds % 60)
    
    if (hours > 0) {
      return `${hours}h ${minutes}m ${secs}s`
    } else if (minutes > 0) {
      return `${minutes}m ${secs}s`
    } else {
      return `${secs}s`
    }
  }

  const formatDistance = (meters: number) => {
    if (meters >= 1000) {
      return `${(meters / 1000).toFixed(2)} km`
    } else {
      return `${meters.toFixed(2)} m`
    }
  }

  const formatPercentage = (value: number) => {
    return `${(value * 100).toFixed(1)}%`
  }

  return (
    <div className="card">
      <div className="flex items-center space-x-3 mb-6">
        <BarChart3 className="h-5 w-5 text-primary-600" />
        <h2 className="text-xl font-semibold text-gray-900">Performance Metrics</h2>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        {/* Goals Completed */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="bg-gradient-to-br from-green-50 to-green-100 rounded-lg p-4 border border-green-200"
        >
          <div className="flex items-center justify-between mb-2">
            <Target className="h-8 w-8 text-green-600" />
            <span className="text-2xl font-bold text-green-700">{metrics.goals_completed}</span>
          </div>
          <h3 className="text-sm font-medium text-green-800">Goals Completed</h3>
          <p className="text-xs text-green-600 mt-1">Total successful missions</p>
        </motion.div>

        {/* Success Rate */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.1 }}
          className="bg-gradient-to-br from-blue-50 to-blue-100 rounded-lg p-4 border border-blue-200"
        >
          <div className="flex items-center justify-between mb-2">
            <TrendingUp className="h-8 w-8 text-blue-600" />
            <span className="text-2xl font-bold text-blue-700">{formatPercentage(metrics.success_rate)}</span>
          </div>
          <h3 className="text-sm font-medium text-blue-800">Success Rate</h3>
          <p className="text-xs text-blue-600 mt-1">Mission completion rate</p>
        </motion.div>

        {/* Distance Traveled */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2 }}
          className="bg-gradient-to-br from-purple-50 to-purple-100 rounded-lg p-4 border border-purple-200"
        >
          <div className="flex items-center justify-between mb-2">
            <Route className="h-8 w-8 text-purple-600" />
            <span className="text-2xl font-bold text-purple-700">{formatDistance(metrics.distance_traveled)}</span>
          </div>
          <h3 className="text-sm font-medium text-purple-800">Distance Traveled</h3>
          <p className="text-xs text-purple-600 mt-1">Total path length</p>
        </motion.div>

        {/* System Uptime */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.3 }}
          className="bg-gradient-to-br from-orange-50 to-orange-100 rounded-lg p-4 border border-orange-200"
        >
          <div className="flex items-center justify-between mb-2">
            <Clock className="h-8 w-8 text-orange-600" />
            <span className="text-2xl font-bold text-orange-700">{formatTime(metrics.system_uptime)}</span>
          </div>
          <h3 className="text-sm font-medium text-orange-800">System Uptime</h3>
          <p className="text-xs text-orange-600 mt-1">Time since startup</p>
        </motion.div>
      </div>

      {/* Additional Metrics */}
      <div className="mt-6 grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* Path Replans */}
        <div className="bg-gray-50 rounded-lg p-4">
          <div className="flex items-center space-x-3 mb-3">
            <div className="w-2 h-2 bg-yellow-500 rounded-full"></div>
            <h3 className="font-medium text-gray-900">Path Replans</h3>
          </div>
          <div className="flex items-baseline space-x-2">
            <span className="text-3xl font-bold text-gray-900">{metrics.total_replans}</span>
            <span className="text-sm text-gray-600">replans</span>
          </div>
          <p className="text-xs text-gray-500 mt-1">
            Total path replanning events due to obstacles or goal changes
          </p>
        </div>

        {/* Performance Summary */}
        <div className="bg-gray-50 rounded-lg p-4">
          <div className="flex items-center space-x-3 mb-3">
            <div className="w-2 h-2 bg-green-500 rounded-full"></div>
            <h3 className="font-medium text-gray-900">Performance Summary</h3>
          </div>
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-gray-600">Avg. Speed:</span>
              <span className="font-medium">
                {metrics.distance_traveled > 0 && metrics.system_uptime > 0
                  ? formatDistance(metrics.distance_traveled / metrics.system_uptime) + '/s'
                  : '0.00 m/s'
                }
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600">Goals/Hour:</span>
              <span className="font-medium">
                {metrics.system_uptime > 0
                  ? (metrics.goals_completed / (metrics.system_uptime / 3600)).toFixed(1)
                  : '0.0'
                }
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600">Replan Rate:</span>
              <span className="font-medium">
                {metrics.system_uptime > 0
                  ? (metrics.total_replans / (metrics.system_uptime / 60)).toFixed(1) + '/min'
                  : '0.0/min'
                }
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Status Indicators */}
      <div className="mt-6 flex flex-wrap gap-2">
        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
          metrics.success_rate > 0.9 ? 'bg-green-100 text-green-800' :
          metrics.success_rate > 0.7 ? 'bg-yellow-100 text-yellow-800' :
          'bg-red-100 text-red-800'
        }`}>
          {metrics.success_rate > 0.9 ? '✓ Excellent' :
           metrics.success_rate > 0.7 ? '⚠ Good' : '✗ Needs Attention'}
        </span>
        
        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
          metrics.total_replans < 10 ? 'bg-green-100 text-green-800' :
          metrics.total_replans < 50 ? 'bg-yellow-100 text-yellow-800' :
          'bg-red-100 text-red-800'
        }`}>
          {metrics.total_replans < 10 ? '✓ Stable Navigation' :
           metrics.total_replans < 50 ? '⚠ Some Replans' : '✗ Frequent Replans'}
        </span>
        
        <span className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
          metrics.system_uptime > 3600 ? 'bg-green-100 text-green-800' :
          metrics.system_uptime > 600 ? 'bg-yellow-100 text-yellow-800' :
          'bg-blue-100 text-blue-800'
        }`}>
          {metrics.system_uptime > 3600 ? '✓ Long Running' :
           metrics.system_uptime > 600 ? '⚠ Medium Uptime' : '🔄 Recently Started'}
        </span>
      </div>
    </div>
  )
}

